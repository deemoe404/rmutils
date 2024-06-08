#include "opencv2/opencv.hpp"
#include "include/progressbar.hpp"

int main(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] << " <input file name> <frames per second> <single output frame limit>" << std::endl;
        return -1;
    }

    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened())
    {
        return -1;
    }

    int fps = std::stoi(argv[2]);
    int limit = std::stoi(argv[3]);
    cv::Size size((int)cap.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    cv::VideoWriter writer;
    progressbar bar(limit);
    int file_index = 0;
    int frame_index = 0;
    cv::Mat frame;
    while (cap.read(frame))
    {
        if (frame_index % limit == 0)
        {
            if (writer.isOpened())
            {
                writer.release();
            }
            std::string file_name = "output_" + std::to_string(file_index) + ".avi";
            writer.open(file_name, cv::VideoWriter::fourcc('F', 'F', 'V', '1'), fps, size);
            std::cout << std::endl
                      << "New file created: " << file_name << std::endl;
            bar.reset();
            file_index++;
        }
        writer.write(frame);
        bar.update();
        frame_index++;
    }
    writer.release();

    return 0;
}
