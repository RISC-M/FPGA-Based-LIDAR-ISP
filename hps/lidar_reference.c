#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#define PORT 2369
#define PAYLOAD_SIZE 1206
#define GRID_SIZE 100
#define GRID_CENTER 50
#define AZIMUTH_WRAP_THRESHOLD 18000
#define SOCKET_RECEIVE_BUFFER (4 * 1024 * 1024)

#define DISTANCE_RESOLUTION_M 0.002
#define GRID_RESOLUTION_M 0.10
#define DEFAULT_SENSOR_HEIGHT_M 0.50
#define DEFAULT_MIN_OBSTACLE_HEIGHT_M 0.10
#define DEFAULT_MAX_OBSTACLE_HEIGHT_M 2.50
#define FIRING_SEQUENCE_US 55.296
#define LASER_CHANNEL_US 2.304
#define DATA_BLOCK_US 110.592
#define MAX_POINTS_PER_ROTATION 50000
#define RANSAC_ITERATIONS 250
#define GROUND_INLIER_DISTANCE_M 0.08
#define MIN_GROUND_INLIERS 100
#define MAX_GROUND_TILT_DEGREES 35.0
#define GROUND_FIT_RADIUS_M 8.0

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    double x;
    double y;
    double z;
} Point3D;

typedef struct {
    double a;
    double b;
    double c;
    double d;
    unsigned int inliers;
    int estimated;
} GroundPlane;

static const double elevation_degrees[16] = {
    -15.0, 1.0, -13.0, 3.0,
    -11.0, 5.0, -9.0, 7.0,
    -7.0, 9.0, -5.0, 11.0,
    -3.0, 13.0, -1.0, 15.0
};

static volatile sig_atomic_t stop_requested = 0;
static int terminal_active = 0;
static Point3D point_buffer[MAX_POINTS_PER_ROTATION];
static uint32_t ground_candidate_indices[MAX_POINTS_PER_ROTATION];

static void request_stop(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static void restore_terminal(void)
{
    if (terminal_active) {
        static const char sequence[] = "\033[?25h\033[?1049l";
        (void)write(STDOUT_FILENO, sequence, sizeof(sequence) - 1);
        terminal_active = 0;
    }
}

static void initialize_terminal(void)
{
    if (!isatty(STDOUT_FILENO)) {
        return;
    }

    static const char sequence[] =
        "\033[?1049h"
        "\033[2J"
        "\033[H"
        "\033[?25l";
    (void)write(STDOUT_FILENO, sequence, sizeof(sequence) - 1);
    terminal_active = 1;
}

static int transform_point(
    uint16_t distance_raw,
    double azimuth_degrees,
    uint8_t laser_id,
    Point3D *point)
{
    if (distance_raw == 0 || laser_id >= 16) {
        return 0;
    }

    double distance_m = distance_raw * DISTANCE_RESOLUTION_M;
    double azimuth_radians = azimuth_degrees * (M_PI / 180.0);
    double elevation_radians =
        elevation_degrees[laser_id] * (M_PI / 180.0);

    double horizontal_distance = distance_m * cos(elevation_radians);
    point->x = horizontal_distance * sin(azimuth_radians);
    point->y = horizontal_distance * cos(azimuth_radians);
    point->z = distance_m * sin(elevation_radians);
    return 1;
}

static double point_plane_distance(
    const GroundPlane *plane,
    const Point3D *point)
{
    return plane->a * point->x +
           plane->b * point->y +
           plane->c * point->z +
           plane->d;
}

static int plane_from_three_points(
    const Point3D *p0,
    const Point3D *p1,
    const Point3D *p2,
    GroundPlane *plane)
{
    double ux = p1->x - p0->x;
    double uy = p1->y - p0->y;
    double uz = p1->z - p0->z;
    double vx = p2->x - p0->x;
    double vy = p2->y - p0->y;
    double vz = p2->z - p0->z;

    double a = uy * vz - uz * vy;
    double b = uz * vx - ux * vz;
    double c = ux * vy - uy * vx;
    double norm = sqrt(a * a + b * b + c * c);
    if (norm < 1e-9) {
        return 0;
    }

    a /= norm;
    b /= norm;
    c /= norm;
    if (c < 0.0) {
        a = -a;
        b = -b;
        c = -c;
    }

    plane->a = a;
    plane->b = b;
    plane->c = c;
    plane->d = -(a * p0->x + b * p0->y + c * p0->z);
    plane->inliers = 0;
    plane->estimated = 1;
    return 1;
}

static uint32_t next_random(uint32_t *state)
{
    uint32_t value = *state;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    *state = value;
    return value;
}

static int solve_three_by_three(
    double matrix[3][3],
    double vector[3],
    double solution[3])
{
    double augmented[3][4];
    for (int row = 0; row < 3; row++) {
        for (int column = 0; column < 3; column++) {
            augmented[row][column] = matrix[row][column];
        }
        augmented[row][3] = vector[row];
    }

    for (int pivot = 0; pivot < 3; pivot++) {
        int best_row = pivot;
        for (int row = pivot + 1; row < 3; row++) {
            if (fabs(augmented[row][pivot]) >
                fabs(augmented[best_row][pivot])) {
                best_row = row;
            }
        }
        if (fabs(augmented[best_row][pivot]) < 1e-10) {
            return 0;
        }
        if (best_row != pivot) {
            for (int column = pivot; column < 4; column++) {
                double temporary = augmented[pivot][column];
                augmented[pivot][column] =
                    augmented[best_row][column];
                augmented[best_row][column] = temporary;
            }
        }

        double divisor = augmented[pivot][pivot];
        for (int column = pivot; column < 4; column++) {
            augmented[pivot][column] /= divisor;
        }
        for (int row = 0; row < 3; row++) {
            if (row == pivot) {
                continue;
            }
            double factor = augmented[row][pivot];
            for (int column = pivot; column < 4; column++) {
                augmented[row][column] -=
                    factor * augmented[pivot][column];
            }
        }
    }

    for (int row = 0; row < 3; row++) {
        solution[row] = augmented[row][3];
    }
    return 1;
}

static void refine_ground_plane(
    GroundPlane *plane,
    const Point3D *points,
    unsigned int candidate_count)
{
    double matrix[3][3] = {{0}};
    double vector[3] = {0};
    unsigned int inliers = 0;

    for (unsigned int candidate = 0;
         candidate < candidate_count;
         candidate++) {
        const Point3D *point =
            &points[ground_candidate_indices[candidate]];
        if (fabs(point_plane_distance(plane, point)) >
            GROUND_INLIER_DISTANCE_M) {
            continue;
        }

        matrix[0][0] += point->x * point->x;
        matrix[0][1] += point->x * point->y;
        matrix[0][2] += point->x;
        matrix[1][1] += point->y * point->y;
        matrix[1][2] += point->y;
        matrix[2][2] += 1.0;
        vector[0] += point->x * point->z;
        vector[1] += point->y * point->z;
        vector[2] += point->z;
        inliers++;
    }

    matrix[1][0] = matrix[0][1];
    matrix[2][0] = matrix[0][2];
    matrix[2][1] = matrix[1][2];

    double solution[3];
    if (inliers < MIN_GROUND_INLIERS ||
        !solve_three_by_three(matrix, vector, solution)) {
        return;
    }

    /* Least-squares surface z = px + qy + r. */
    double a = -solution[0];
    double b = -solution[1];
    double c = 1.0;
    double d = -solution[2];
    double norm = sqrt(a * a + b * b + c * c);
    plane->a = a / norm;
    plane->b = b / norm;
    plane->c = c / norm;
    plane->d = d / norm;
    plane->inliers = inliers;
}

static GroundPlane estimate_ground_plane(
    const Point3D *points,
    unsigned int point_count,
    double sensor_height_m)
{
    GroundPlane fallback = {
        0.0, 0.0, 1.0, sensor_height_m, 0, 0
    };
    unsigned int candidate_count = 0;
    for (unsigned int index = 0; index < point_count; index++) {
        double radius_squared =
            points[index].x * points[index].x +
            points[index].y * points[index].y;
        if (points[index].z < 0.25 &&
            radius_squared <=
                GROUND_FIT_RADIUS_M * GROUND_FIT_RADIUS_M) {
            ground_candidate_indices[candidate_count++] = index;
        }
    }
    if (candidate_count < MIN_GROUND_INLIERS) {
        return fallback;
    }

    GroundPlane best = fallback;
    double minimum_up_component =
        cos(MAX_GROUND_TILT_DEGREES * (M_PI / 180.0));
    uint32_t random_state = 0x6D2B79F5U ^ point_count;

    for (int iteration = 0; iteration < RANSAC_ITERATIONS; iteration++) {
        unsigned int i0 =
            next_random(&random_state) % candidate_count;
        unsigned int i1 =
            next_random(&random_state) % candidate_count;
        unsigned int i2 =
            next_random(&random_state) % candidate_count;
        if (i0 == i1 || i0 == i2 || i1 == i2) {
            continue;
        }

        GroundPlane candidate;
        if (!plane_from_three_points(
                &points[ground_candidate_indices[i0]],
                &points[ground_candidate_indices[i1]],
                &points[ground_candidate_indices[i2]],
                &candidate) ||
            candidate.c < minimum_up_component) {
            continue;
        }

        double inferred_height = candidate.d / candidate.c;
        if (inferred_height < 0.05 ||
            fabs(inferred_height - sensor_height_m) > 0.75) {
            continue;
        }

        unsigned int inliers = 0;
        for (unsigned int index = 0;
             index < candidate_count;
             index++) {
            const Point3D *point =
                &points[ground_candidate_indices[index]];
            if (fabs(point_plane_distance(&candidate, point)) <=
                GROUND_INLIER_DISTANCE_M) {
                inliers++;
            }
        }
        if (inliers > best.inliers) {
            candidate.inliers = inliers;
            best = candidate;
        }
    }

    if (best.inliers < MIN_GROUND_INLIERS) {
        return fallback;
    }
    refine_ground_plane(&best, points, candidate_count);
    return best;
}

static unsigned int build_occupancy_grid(
    const Point3D *points,
    unsigned int point_count,
    const GroundPlane *ground,
    double min_obstacle_height_m,
    double max_obstacle_height_m,
    uint16_t grid[GRID_SIZE][GRID_SIZE])
{
    memset(grid, 0, sizeof(uint16_t) * GRID_SIZE * GRID_SIZE);
    unsigned int accepted = 0;

    for (unsigned int index = 0; index < point_count; index++) {
        double height = point_plane_distance(ground, &points[index]);
        if (height < min_obstacle_height_m ||
            height > max_obstacle_height_m) {
            continue;
        }

        int x = (int)floor(points[index].x / GRID_RESOLUTION_M) +
            GRID_CENTER;
        int y = (int)floor(points[index].y / GRID_RESOLUTION_M) +
            GRID_CENTER;
        if (x < 0 || x >= GRID_SIZE || y < 0 || y >= GRID_SIZE) {
            continue;
        }

        if (grid[y][x] < UINT16_MAX) {
            grid[y][x]++;
        }
        accepted++;
    }
    return accepted;
}

static uint8_t braille_bit(int dot_x, int dot_y)
{
    static const uint8_t bits[4][2] = {
        {1U << 0, 1U << 3},
        {1U << 1, 1U << 4},
        {1U << 2, 1U << 5},
        {1U << 6, 1U << 7}
    };
    return bits[dot_y][dot_x];
}

static void append_braille(
    char *frame,
    size_t frame_size,
    size_t *frame_index,
    uint8_t pattern)
{
    if (pattern == 0) {
        if (*frame_index + 1 < frame_size) {
            frame[(*frame_index)++] = ' ';
        }
        return;
    }

    uint32_t codepoint = 0x2800U + pattern;
    if (*frame_index + 3 < frame_size) {
        frame[(*frame_index)++] = (char)(0xE0U | (codepoint >> 12));
        frame[(*frame_index)++] =
            (char)(0x80U | ((codepoint >> 6) & 0x3FU));
        frame[(*frame_index)++] = (char)(0x80U | (codepoint & 0x3FU));
    }
}

static void get_display_size(int *width, int *height)
{
    struct winsize terminal_size;
    int columns = 80;
    int rows = 24;

    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &terminal_size) == 0) {
        if (terminal_size.ws_col > 0) {
            columns = terminal_size.ws_col;
        }
        if (terminal_size.ws_row > 0) {
            rows = terminal_size.ws_row;
        }
    }

    *width = columns < (GRID_SIZE / 2) ? columns : (GRID_SIZE / 2);
    *height = rows > 3 ? rows - 3 : 1;
    if (*height > (GRID_SIZE / 4)) {
        *height = GRID_SIZE / 4;
    }
}

static unsigned int count_occupied_cells(
    const uint16_t grid[GRID_SIZE][GRID_SIZE])
{
    unsigned int occupied = 0;

    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] > 0) {
                occupied++;
            }
        }
    }

    return occupied;
}

static void render_grid(
    const uint16_t grid[GRID_SIZE][GRID_SIZE],
    unsigned long frame_number,
    unsigned int accepted_points,
    const GroundPlane *ground)
{
    char frame[12000];
    int display_width;
    int display_height;
    get_display_size(&display_width, &display_height);

    unsigned int occupied_cells = count_occupied_cells(grid);
    double ground_tilt_degrees =
        acos(fmax(-1.0, fmin(1.0, ground->c))) * (180.0 / M_PI);
    int written = snprintf(
        frame,
        sizeof(frame),
        "\033[H"
        "PHYSICAL CPU OCCUPANCY GRID - Ctrl-C to exit\033[K\r\n"
        "F:%lu pts:%u cells:%u ground:%s n:%u tilt:%.1f "
        "Braille:%dx%d\033[K\r\n",
        frame_number,
        accepted_points,
        occupied_cells,
        ground->estimated ? "fit" : "fallback",
        ground->inliers,
        ground_tilt_degrees,
        display_width * 2,
        display_height * 4);
    if (written < 0 || (size_t)written >= sizeof(frame)) {
        return;
    }

    size_t frame_index = (size_t)written;
    int virtual_width = display_width * 2;
    int virtual_height = display_height * 4;
    for (int display_y = 0; display_y < display_height; display_y++) {
        for (int display_x = 0;
             display_x < display_width;
             display_x++) {
            uint8_t pattern = 0;

            for (int dot_y = 0; dot_y < 4; dot_y++) {
                int virtual_y = display_y * 4 + dot_y;
                int source_y_begin =
                    (virtual_y * GRID_SIZE) / virtual_height;
                int source_y_end =
                    ((virtual_y + 1) * GRID_SIZE) / virtual_height;

                for (int dot_x = 0; dot_x < 2; dot_x++) {
                    int virtual_x = display_x * 2 + dot_x;
                    int source_x_begin =
                        (virtual_x * GRID_SIZE) / virtual_width;
                    int source_x_end =
                        ((virtual_x + 1) * GRID_SIZE) / virtual_width;
                    int occupied = 0;

                    for (int source_y_index = source_y_begin;
                         source_y_index < source_y_end && !occupied;
                         source_y_index++) {
                        int source_y =
                            GRID_SIZE - 1 - source_y_index;
                        for (int source_x = source_x_begin;
                             source_x < source_x_end;
                             source_x++) {
                            if (grid[source_y][source_x] > 0) {
                                occupied = 1;
                                break;
                            }
                        }
                    }

                    if (occupied) {
                        pattern |= braille_bit(dot_x, dot_y);
                    }
                }
            }

            append_braille(
                frame,
                sizeof(frame),
                &frame_index,
                pattern);
        }

        if (frame_index + 5 < sizeof(frame)) {
            memcpy(frame + frame_index, "\033[K", 3);
            frame_index += 3;
            if (display_y + 1 < display_height) {
                frame[frame_index++] = '\r';
                frame[frame_index++] = '\n';
            }
        }
    }

    if (frame_index + 7 < sizeof(frame)) {
        memcpy(frame + frame_index, "\033[J\033[H", 6);
        frame_index += 6;
    }

    (void)fwrite(frame, 1, frame_index, stdout);
    (void)fflush(stdout);
}

static int dump_grid(
    const char *path,
    const uint16_t grid[GRID_SIZE][GRID_SIZE],
    unsigned long frame_number)
{
    FILE *output = fopen(path, "w");
    if (output == NULL) {
        return -1;
    }

    fprintf(output, "# frame %lu\n", frame_number);
    fprintf(output, "# x y occupancy\n");
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] > 0) {
                fprintf(
                    output,
                    "%d %d %u\n",
                    x,
                    y,
                    (unsigned int)grid[y][x]);
            }
        }
    }

    return fclose(output);
}

static int parse_double_option(const char *text, double *value)
{
    char *end = NULL;
    errno = 0;
    double parsed = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
        return -1;
    }

    *value = parsed;
    return 0;
}

static void print_usage(const char *program)
{
    fprintf(
        stderr,
        "Usage: %s [options]\n"
        "  --sensor-height M   LiDAR origin above ground (default %.2f)\n"
        "  --min-height M      Lowest obstacle height (default %.2f)\n"
        "  --max-height M      Highest obstacle height (default %.2f)\n"
        "  --dump FILE         Overwrite FILE with each completed grid\n",
        program,
        DEFAULT_SENSOR_HEIGHT_M,
        DEFAULT_MIN_OBSTACLE_HEIGHT_M,
        DEFAULT_MAX_OBSTACLE_HEIGHT_M);
}

int main(int argc, char **argv)
{
    const char *dump_path = NULL;
    double sensor_height_m = DEFAULT_SENSOR_HEIGHT_M;
    double min_obstacle_height_m = DEFAULT_MIN_OBSTACLE_HEIGHT_M;
    double max_obstacle_height_m = DEFAULT_MAX_OBSTACLE_HEIGHT_M;

    for (int argument = 1; argument < argc; argument++) {
        if (argument + 1 >= argc) {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }

        const char *option = argv[argument];
        const char *value = argv[++argument];
        if (strcmp(option, "--dump") == 0) {
            dump_path = value;
        } else if (strcmp(option, "--sensor-height") == 0) {
            if (parse_double_option(value, &sensor_height_m) == -1) {
                fprintf(stderr, "Invalid sensor height: %s\n", value);
                return EXIT_FAILURE;
            }
        } else if (strcmp(option, "--min-height") == 0) {
            if (parse_double_option(value, &min_obstacle_height_m) == -1) {
                fprintf(stderr, "Invalid minimum height: %s\n", value);
                return EXIT_FAILURE;
            }
        } else if (strcmp(option, "--max-height") == 0) {
            if (parse_double_option(value, &max_obstacle_height_m) == -1) {
                fprintf(stderr, "Invalid maximum height: %s\n", value);
                return EXIT_FAILURE;
            }
        } else {
            fprintf(stderr, "Unknown option: %s\n", option);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (sensor_height_m < 0.0 ||
        min_obstacle_height_m > max_obstacle_height_m) {
        fprintf(stderr, "Invalid height configuration\n");
        return EXIT_FAILURE;
    }

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("ERROR: socket creation failed");
        return EXIT_FAILURE;
    }

    int reuse_address = 1;
    if (setsockopt(
            sockfd,
            SOL_SOCKET,
            SO_REUSEADDR,
            &reuse_address,
            sizeof(reuse_address)) == -1) {
        perror("ERROR: setsockopt failed");
        close(sockfd);
        return EXIT_FAILURE;
    }

    int receive_buffer_size = SOCKET_RECEIVE_BUFFER;
    if (setsockopt(
            sockfd,
            SOL_SOCKET,
            SO_RCVBUF,
            &receive_buffer_size,
            sizeof(receive_buffer_size)) == -1) {
        perror("WARNING: could not enlarge UDP receive buffer");
    }

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(PORT);

    if (bind(
            sockfd,
            (const struct sockaddr *)&address,
            sizeof(address)) == -1) {
        perror("ERROR: bind failed");
        close(sockfd);
        return EXIT_FAILURE;
    }

    struct sigaction stop_action;
    memset(&stop_action, 0, sizeof(stop_action));
    stop_action.sa_handler = request_stop;
    sigemptyset(&stop_action.sa_mask);
    if (sigaction(SIGINT, &stop_action, NULL) == -1 ||
        sigaction(SIGTERM, &stop_action, NULL) == -1) {
        perror("WARNING: could not install signal handlers");
    }

    if (atexit(restore_terminal) != 0) {
        fprintf(stderr, "WARNING: could not register terminal cleanup\n");
    }
    initialize_terminal();

    uint8_t packet[PAYLOAD_SIZE];
    uint16_t grid[GRID_SIZE][GRID_SIZE] = {{0}};
    int previous_azimuth = -1;
    int recent_block_azimuth_delta = 40;
    int collecting_complete_rotation = 0;
    unsigned long frame_number = 0;
    unsigned int point_count = 0;
    int exit_status = EXIT_SUCCESS;

    while (!stop_requested) {
        ssize_t received = recvfrom(
            sockfd,
            packet,
            sizeof(packet),
            0,
            NULL,
            NULL);
        if (received == -1) {
            if (errno == EINTR && stop_requested) {
                break;
            }
            perror("ERROR: recvfrom failed");
            exit_status = EXIT_FAILURE;
            break;
        }
        if (received != PAYLOAD_SIZE) {
            continue;
        }

        for (int block = 0; block < 12; block++) {
            int offset = block * 100;
            uint16_t flag =
                (uint16_t)packet[offset] |
                ((uint16_t)packet[offset + 1] << 8);
            if (flag != 0xEEFF) {
                continue;
            }

            uint16_t azimuth =
                (uint16_t)packet[offset + 2] |
                ((uint16_t)packet[offset + 3] << 8);
            int azimuth_wrapped =
                previous_azimuth != -1 &&
                previous_azimuth > (int)azimuth &&
                (previous_azimuth - (int)azimuth) >
                    AZIMUTH_WRAP_THRESHOLD;

            if (azimuth_wrapped) {
                /*
                 * The first wrap only establishes a clean revolution
                 * boundary; data collected before it was a partial frame.
                 */
                if (collecting_complete_rotation) {
                    GroundPlane ground = estimate_ground_plane(
                        point_buffer,
                        point_count,
                        sensor_height_m);
                    unsigned int accepted_points =
                        build_occupancy_grid(
                            point_buffer,
                            point_count,
                            &ground,
                            min_obstacle_height_m,
                            max_obstacle_height_m,
                            grid);
                    frame_number++;
                    render_grid(
                        grid,
                        frame_number,
                        accepted_points,
                        &ground);
                    if (dump_path != NULL &&
                        dump_grid(dump_path, grid, frame_number) == -1) {
                        restore_terminal();
                        perror("WARNING: could not write grid dump");
                        initialize_terminal();
                    }
                } else {
                    collecting_complete_rotation = 1;
                }

                memset(grid, 0, sizeof(grid));
                point_count = 0;
            }
            previous_azimuth = azimuth;

            /*
             * A block covers 110.592 us but only stores its starting
             * azimuth. Estimate angular travel from the following block.
             * For the final block in a packet, reuse the most recent delta.
             */
            int block_azimuth_delta = recent_block_azimuth_delta;
            if (block + 1 < 12) {
                int next_offset = (block + 1) * 100;
                uint16_t next_flag =
                    (uint16_t)packet[next_offset] |
                    ((uint16_t)packet[next_offset + 1] << 8);
                if (next_flag == 0xEEFF) {
                    uint16_t next_azimuth =
                        (uint16_t)packet[next_offset + 2] |
                        ((uint16_t)packet[next_offset + 3] << 8);
                    int measured_delta =
                        ((int)next_azimuth - (int)azimuth + 36000) % 36000;
                    if (measured_delta > 0 && measured_delta < 1000) {
                        block_azimuth_delta = measured_delta;
                        recent_block_azimuth_delta = measured_delta;
                    }
                }
            }

            for (int laser_id = 0; laser_id < 16; laser_id++) {
                int point0 = offset + 4 + (laser_id * 3);
                int point1 = offset + 4 + ((laser_id + 16) * 3);

                uint16_t distance0 =
                    (uint16_t)packet[point0] |
                    ((uint16_t)packet[point0 + 1] << 8);
                uint16_t distance1 =
                    (uint16_t)packet[point1] |
                    ((uint16_t)packet[point1 + 1] << 8);

                double firing0_offset_us =
                    laser_id * LASER_CHANNEL_US;
                double firing1_offset_us =
                    FIRING_SEQUENCE_US + laser_id * LASER_CHANNEL_US;
                double azimuth0_degrees = fmod(
                    ((double)azimuth +
                     block_azimuth_delta *
                         (firing0_offset_us / DATA_BLOCK_US)) /
                        100.0,
                    360.0);
                double azimuth1_degrees = fmod(
                    ((double)azimuth +
                     block_azimuth_delta *
                         (firing1_offset_us / DATA_BLOCK_US)) /
                        100.0,
                    360.0);

                /*
                 * packet[point0 + 2] and packet[point1 + 2] are calibrated
                 * reflectivity bytes. Reflectivity describes the target
                 * surface, not its geometry, so it is intentionally not
                 * used as an obstacle-distance or height filter.
                 */
                if (point_count < MAX_POINTS_PER_ROTATION &&
                    transform_point(
                        distance0,
                        azimuth0_degrees,
                        (uint8_t)laser_id,
                        &point_buffer[point_count])) {
                    point_count++;
                }
                if (point_count < MAX_POINTS_PER_ROTATION &&
                    transform_point(
                        distance1,
                        azimuth1_degrees,
                        (uint8_t)laser_id,
                        &point_buffer[point_count])) {
                    point_count++;
                }
            }
        }
    }

    close(sockfd);
    return exit_status;
}
