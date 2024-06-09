/// To keep minimum dependencies, I used cvui library for GUI.
/// cvui is a header-only library that uses OpenCV under the hood.
/// You can find the library at https://github.com/Dovyski/cvui/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <random>
#include <ctime>

#define CVUI_IMPLEMENTATION
#include "include/cvui.h"

#define WINDOW_NAME "RoboBraver labelling tool (Press ESC to close the window)"
#define SCREEN_DPI 192
#define SCALING_FACTOR 0.4 * SCREEN_DPI / 96
#define PANEL_HEIGHT 512

std::vector<cv::Scalar> color_map;

/**
 * @brief Apply image processing to the frame
 *
 * Change this function to apply different image processing techniques you need.
 *
 * @param frame     Original frame
 * @param threshold Threshold value for binary thresholding
 * @return cv::Mat Processed frame
 */
cv::Mat image_processing(cv::Mat frame, int threshold = 50)
{
    cv::Mat gray;
    gray = frame.clone();

    // Todo: Pass in Gray image to avoid color conversion (let the coneversion be done in GUI related functions)
    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, gray, threshold, 255, cv::THRESH_BINARY);
    cv::cvtColor(gray, gray, cv::COLOR_GRAY2BGR);
    return gray;
}

/**
 * @brief Get contours from the binary image
 *
 * Change this function to apply different contour detection techniques you need.
 * (such as acreage filtering, convex hull, etc.)
 *
 * @param input Binary image
 * @return std::vector<std::vector<cv::Point>> Contours
 */
std::vector<std::vector<cv::Point>> get_contours(cv::Mat &input)
{
    cv::Mat binary;
    std::vector<std::vector<cv::Point>> result;
    cv::cvtColor(input, binary, cv::COLOR_BGR2GRAY);
    cv::findContours(binary, result, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    return result;
}

std::string get_date()
{
    auto now = std::chrono::system_clock::now();

    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y-%m-%d_%H:%M:%S");
    return oss.str();
}

cv::Scalar hslToRgb(float h, float s, float l)
{
    auto hueToRgb = [](float p, float q, float t)
    {
        if (t < 0.0f)
            t += 1.0f;
        if (t > 1.0f)
            t -= 1.0f;
        if (t < 1.0f / 6.0f)
            return p + (q - p) * 6.0f * t;
        if (t < 1.0f / 2.0f)
            return q;
        if (t < 2.0f / 3.0f)
            return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
        return p;
    };

    float r, g, b;

    if (s == 0.0f)
    {
        r = g = b = l;
    }
    else
    {
        float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
        float p = 2.0f * l - q;
        r = hueToRgb(p, q, h + 1.0f / 3.0f);
        g = hueToRgb(p, q, h);
        b = hueToRgb(p, q, h - 1.0f / 3.0f);
    }

    return cv::Scalar(b * 255.0f, g * 255.0f, r * 255.0f);
}

std::vector<cv::Scalar> generateColors(int numColors)
{
    std::vector<cv::Scalar> colors;
    float goldenRatioConjugate = 0.61803398875f;
    float h = 0.0f;

    for (int i = 0; i < numColors; ++i)
    {
        h += goldenRatioConjugate;
        h = std::fmod(h, 1.0f);
        colors.push_back(hslToRgb(h, 0.5f, 0.6f));
    }

    return colors;
}

std::vector<std::vector<cv::Point>> select_contour(
    std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> &saved_data,
    std::vector<int> contour_index, int frame_index)
{
    std::vector<std::vector<cv::Point>> result;

    for (auto index : contour_index)
    {
        for (auto &frame_contours : saved_data)
        {
            if (frame_contours.first == frame_index)
            {
                for (auto &contour : frame_contours.second)
                {
                    if (contour.first == index)
                    {
                        result.push_back(contour.second);
                    }
                }
            }
        }
    }
    return result;
}

void draw_canvas(cv::Mat &canvas, cv::Mat &frame, cv::Mat &processed, int threshold = 100, int current_frame = 0)
{
    canvas = cv::Scalar(49, 52, 49);

    cv::Mat left(canvas, cv::Rect(0, 0, frame.cols, frame.rows));
    processed.copyTo(left);
    cv::Mat right(canvas, cv::Rect(frame.cols, 0, frame.cols, frame.rows));
    frame.copyTo(right);

    cvui::beginRow(canvas, 20 * SCALING_FACTOR, frame.rows + 20 * SCALING_FACTOR, -1, -1);

    cvui::beginColumn(-1, -1);
    cvui::text("Keyboard Shortcuts", SCALING_FACTOR * 1.5);
    cvui::space(25);
    cvui::text("    Increase threshold:", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("    Decrease threshold:", SCALING_FACTOR);
    cvui::space(40);
    cvui::text("    Previous frame:", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("    Next frame:", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("    Next 100 frame:", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("    Previous 100 frame:", SCALING_FACTOR);
    cvui::space(40);
    cvui::text("    Save pair:", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("    Reset selection:", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("    Remove pair:", SCALING_FACTOR);
    cvui::space(40);
    cvui::text("    Print debug info:", SCALING_FACTOR);
    cvui::endColumn();

    cvui::beginColumn(-1, -1);
    cvui::text("-", SCALING_FACTOR * 1.5);
    cvui::space(25);
    cvui::text("=", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("-", SCALING_FACTOR);
    cvui::space(40);
    cvui::text("[", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("]", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("\\", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("p", SCALING_FACTOR);
    cvui::space(40);
    cvui::text("w", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("q", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("e", SCALING_FACTOR);
    cvui::space(40);
    cvui::text(",", SCALING_FACTOR);
    cvui::endColumn();

    cvui::beginColumn(-1, -1);
    cvui::text("    Current parameters", SCALING_FACTOR * 1.5);
    cvui::space(25);
    cvui::text("        Threshold: ", SCALING_FACTOR);
    cvui::space(15);
    cvui::text("        Frame index: ", SCALING_FACTOR);
    cvui::endColumn();

    cvui::beginColumn(-1, -1);
    cvui::text("-", SCALING_FACTOR * 1.5);
    cvui::space(25);
    cvui::text(std::to_string(threshold), SCALING_FACTOR);
    cvui::space(15);
    cvui::text(std::to_string(current_frame), SCALING_FACTOR);
    cvui::endColumn();

    cvui::beginColumn(-1, -1);
    cvui::text("    Notice", SCALING_FACTOR * 1.5);
    cvui::space(25);
    cvui::text("        * Changing the image processing parameters will reset all labeled data.", SCALING_FACTOR);
    cvui::endColumn();

    cvui::endRow();
}

void visual_piars(
    cv::Mat &canvas,
    std::vector<std::pair<int, std::vector<std::pair<int, int>>>> &positive_pairs,
    std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> &saved_data,
    int current_frame, int offset = 0)
{
    std::vector<std::vector<cv::Point>> contour_1, contour_2;

    std::vector<std::pair<int, int>> pairs;
    for (auto &positive_pair : positive_pairs)
    {
        if (positive_pair.first == current_frame)
        {
            pairs = positive_pair.second;
            break;
        }
    }

    if (pairs.size() <= 0)
    {
        return;
    }

    for (auto &pair : pairs)
    {
        for (auto &frame_contours : saved_data)
        {
            if (frame_contours.first == current_frame)
            {
                for (auto &contour : frame_contours.second)
                {
                    if (contour.first == pair.first)
                    {
                        contour_1 = {contour.second};
                    }
                    if (contour.first == pair.second)
                    {
                        contour_2 = {contour.second};
                    }
                }
                cv::Scalar rand_color = color_map[(pair.first + pair.second) % color_map.size()];
                if (contour_1.size() > 0 && contour_2.size() > 0)
                {
                    cv::drawContours(canvas, {contour_1}, -1, rand_color, 6, cv::FILLED);
                    cv::drawContours(canvas, {contour_2}, -1, rand_color, 6, cv::FILLED);
                    cv::line(canvas, contour_1[0][0], contour_2[0][0], rand_color, 2);

                    cv::drawContours(canvas, {contour_1}, -1, rand_color, 6, cv::FILLED, cv::noArray(), INT_MAX, cv::Point(offset, 0));
                    cv::drawContours(canvas, {contour_2}, -1, rand_color, 6, cv::FILLED, cv::noArray(), INT_MAX, cv::Point(offset, 0));
                    cv::line(canvas, contour_1[0][0] + cv::Point(offset, 0), contour_2[0][0] + cv::Point(offset, 0), rand_color, 2);
                }
            }
        }
    }
}

void visual_hover(
    cv::Mat &canvas,
    std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> &saved_data,
    int hover, int frame_index, std::string tooltips, int offset = 0)
{
    if (hover != -1)
    {
        std::vector<std::vector<cv::Point>> hover_contour = select_contour(saved_data, {hover}, frame_index);
        cv::drawContours(canvas, hover_contour, -1, cv::Scalar(0, 255, 0), 8, cv::FILLED);
        cv::drawContours(canvas, hover_contour, -1, cv::Scalar(0, 255, 0), 8, cv::FILLED, cv::noArray(), INT_MAX, cv::Point(offset, 0));

        cv::putText(canvas, "Contour " + tooltips, hover_contour[0][0], cv::FONT_HERSHEY_SIMPLEX, SCALING_FACTOR, cv::Scalar(0, 255, 0), 1);
    }
}

void save_contours(std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> &saved_data,
                   std::vector<std::vector<cv::Point>> &frame_contours,
                   int frame_index)
{
    bool found = false;
    for (auto &video_countour : saved_data)
    {
        if (video_countour.first == frame_index)
        {
            found = true;
            break;
        }
    }
    if (!found)
    {
        saved_data.push_back({frame_index, {}});

        for (int i = 0; i < frame_contours.size(); i++)
        {
            saved_data.back().second.push_back({i, frame_contours[i]});
        }
    }
}

void print_saved(std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> &saved_data)
{
    for (auto &frame_contours : saved_data)
    {
        std::cout << "Frame " << frame_contours.first << std::endl;
        for (auto &contour : frame_contours.second)
        {
            std::cout << "Contour " << contour.first << " : " << std::endl;
            for (auto &point : contour.second)
            {
                std::cout << point << ", ";
            }
            std::cout << std::endl;
        }
    }
}

void print_positive(std::vector<std::pair<int, std::vector<std::pair<int, int>>>> &positive_pairs)
{
    for (auto &positive_pair : positive_pairs)
    {
        std::cout << "Frame " << positive_pair.first << ": " << std::endl;
        for (auto &pair : positive_pair.second)
        {
            std::cout << pair.first << " - " << pair.second << std::endl;
        }
    }
}

int get_hover(
    std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> &saved_data,
    int mouse_x, int mouse_y, int frame_index)
{
    double min_distance = DBL_MAX;
    int min_index = -1;
    for (auto &frame_contours : saved_data)
    {
        if (frame_contours.first == frame_index)
        {
            for (auto &contour : frame_contours.second)
            {
                for (int j = 0; j < contour.second.size(); j++)
                {
                    double distance = cv::norm(cv::Point(mouse_x, mouse_y) - contour.second[j]);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        min_index = contour.first;
                    }
                }
            }
        }
    }
    return min_index;
}

void write_data(
    std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> &saved_data,
    std::vector<std::pair<int, std::vector<std::pair<int, int>>>> &positive_pairs,
    std::string data_file, std::string positive_file)
{
    std::ofstream ofs(data_file);
    if (!ofs)
    {
        return;
    }

    for (const auto &outer_pair : saved_data)
    {
        ofs << "Frame_" << outer_pair.first << ":" << std::endl;
        for (const auto &inner_pair : outer_pair.second)
        {
            ofs << "Contour_" << inner_pair.first << ": ";
            for (const auto &point : inner_pair.second)
            {
                ofs << point.x << "," << point.y << "; ";
            }
            ofs << std::endl;
        }
        ofs << std::endl;
    }

    ofs.close();

    std::ofstream ofs2(positive_file);
    if (!ofs2)
    {
        return;
    }

    for (const auto &outer_pair : positive_pairs)
    {
        ofs2 << "Frame_" << outer_pair.first << ":" << std::endl;
        for (const auto &inner_pair : outer_pair.second)
        {
            ofs2 << inner_pair.first << "-" << inner_pair.second << std::endl;
        }
        ofs2 << std::endl;
    }

    ofs2.close();
}

void read_data(
    std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> &saved_data,
    std::vector<std::pair<int, std::vector<std::pair<int, int>>>> &positive_pairs,
    std::string data_file, std::string positive_file)
{
    std::ifstream ifs(data_file);
    if (!ifs)
    {
        return;
    }

    saved_data.clear();
    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.find("Frame_") == 0)
        {
            int frame_number = std::stoi(line.substr(6, line.find(':') - 6));
            std::vector<std::pair<int, std::vector<cv::Point>>> contours;

            while (std::getline(ifs, line) && !line.empty())
            {
                if (line.find("Contour_") == 0)
                {
                    int contour_number = std::stoi(line.substr(8, line.find(':') - 8));
                    std::vector<cv::Point> points;

                    std::string points_str = line.substr(line.find(':') + 2);
                    std::stringstream ss(points_str);
                    std::string point_str;

                    while (std::getline(ss, point_str, ';'))
                    {
                        if (point_str.empty() || point_str.find(',') == std::string::npos)
                            continue;
                        int x = std::stoi(point_str.substr(0, point_str.find(',')));
                        int y = std::stoi(point_str.substr(point_str.find(',') + 1));
                        points.emplace_back(x, y);
                    }

                    contours.emplace_back(contour_number, points);
                }
            }

            saved_data.emplace_back(frame_number, contours);
        }
    }

    ifs.close();

    std::ifstream ifs2(positive_file);
    if (!ifs2)
    {
        std::cerr << "Error opening positive file." << std::endl;
        return;
    }

    positive_pairs.clear();
    while (std::getline(ifs2, line))
    {
        if (line.find("Frame_") == 0)
        {
            int frame_number = std::stoi(line.substr(6, line.find(':') - 6));
            std::vector<std::pair<int, int>> pairs;

            while (std::getline(ifs2, line) && !line.empty())
            {
                int hyphen_pos = line.find('-');
                int first = std::stoi(line.substr(0, hyphen_pos));
                int second = std::stoi(line.substr(hyphen_pos + 1));

                pairs.emplace_back(first, second);
            }

            positive_pairs.emplace_back(frame_number, pairs);
        }
    }

    ifs2.close();
}

int main(int argc, const char *argv[])
{
    color_map = generateColors(200);

    cv::VideoCapture cap("../../../output.avi");
    if (!cap.isOpened() || (int)cap.get(cv::CAP_PROP_FRAME_COUNT) == 0)
    {
        std::cerr << "Video file might be missplaced or corrupted, consider using the videofix tool." << std::endl;
        return -1;
    }

    cv::Mat canvas = cv::Mat(cv::Size(
                                 (int)cap.get(cv::CAP_PROP_FRAME_WIDTH) * 2,
                                 (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT) + PANEL_HEIGHT),
                             CV_8UC3);
    int frame_count = (int)cap.get(cv::CAP_PROP_FRAME_COUNT) - 1;

    // all contours in each frame of the video
    std::vector<std::pair<int, std::vector<std::pair<int, std::vector<cv::Point>>>>> video_contours;
    //                |-- first  : int                         -> frame index
    //                |-- second : std::vector<std::pair>      -> all contours in the frame
    //                     |-- first : int                     -> contour index
    //                     |-- second : std::vector<cv::Point> -> contour data

    // all positive pairs in the video
    std::vector<std::pair<int, std::vector<std::pair<int, int>>>> positive_pairs;
    //                |-- first  : int                     -> frame index
    //                |-- second : std::vector<std::pair>  -> all positive pairs in the frame
    //                     |-- item : std::pair            -> positive pair

    cvui::init(WINDOW_NAME);

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat frame, processed;
    int threshold = 100, current_frame = 0;
    cv::Scalar color_contour = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);

    int contour_buffer = -1;
    while (true)
    {
        int hover = get_hover(video_contours, cvui::mouse().x, cvui::mouse().y, current_frame);
        draw_canvas(canvas, frame, processed, threshold, current_frame);
        visual_piars(canvas, positive_pairs, video_contours, current_frame, frame.cols);
        visual_hover(canvas, video_contours, hover, current_frame, contour_buffer == -1 ? "1" : "2", frame.cols);

        cvui::update();
        cvui::imshow(WINDOW_NAME, canvas);

        char key = (char)cv::waitKey(60);
        switch (key)
        {
        case ',':
            print_saved(video_contours);
            print_positive(positive_pairs);
            break;
        case 27:
            return 0;
        case '=':
            threshold = threshold < 255 ? threshold + 1 : 0;
            video_contours.clear();
            positive_pairs.clear();
            break;
        case '-':
            threshold = threshold > 0 ? threshold - 1 : 255;
            video_contours.clear();
            positive_pairs.clear();
            break;
        case '[':
            current_frame = current_frame > 0 ? current_frame - 1 : frame_count;
            break;
        case ']':
            current_frame = current_frame < frame_count ? current_frame + 1 : 0;
            break;
        case '\\':
            current_frame = current_frame + 100 > frame_count ? 0 : current_frame + 100;
            break;
        case 'p':
            current_frame = current_frame - 100 < 0 ? frame_count : current_frame - 100;
            break;
        case 'w':
            if (contour_buffer == -1)
            {
                contour_buffer = hover;
            }
            else
            {
                bool found_frame = false;
                for (auto &positive_pair : positive_pairs)
                {
                    if (positive_pair.first == current_frame)
                    {
                        found_frame = true;
                        break;
                    }
                }
                if (!found_frame)
                {
                    positive_pairs.push_back({current_frame, {}});
                }
                for (auto &positive_pair : positive_pairs)
                {
                    if (positive_pair.first == current_frame)
                    {
                        bool found_contour = false;
                        for (auto &pair : positive_pair.second)
                        {
                            if (pair.first == hover || pair.second == hover ||
                                pair.first == contour_buffer || pair.second == contour_buffer)
                            {
                                found_contour = true;
                                contour_buffer = -1;
                                break;
                            }
                        }
                        if (!found_contour)
                        {
                            positive_pair.second.push_back({contour_buffer, hover});
                        }
                        break;
                    }
                }
                contour_buffer = -1;
            }
            break;
        case 'q':
            if (contour_buffer != -1)
            {
                contour_buffer = -1;
            }
            break;
        case 'e':
            for (auto &positive_pair : positive_pairs)
            {
                if (positive_pair.first == current_frame)
                {
                    for (int i = 0; i < positive_pair.second.size(); i++)
                    {
                        if (positive_pair.second[i].first == hover || positive_pair.second[i].second == hover)
                        {
                            positive_pair.second.erase(positive_pair.second.begin() + i);
                            break;
                        }
                    }
                }
            }
            break;
        case 's':
            std::string date = get_date();
            write_data(video_contours, positive_pairs, date + ".rb_data", date + ".rb_positive");
            break;
        }
        if (key > -1 || frame.empty())
        {
            cap.set(cv::CAP_PROP_POS_FRAMES, current_frame);
            cap.read(frame);
            processed = image_processing(frame, threshold);
            contours = get_contours(processed);
            save_contours(video_contours, contours, current_frame);
        }
    }
    return 0;
}
