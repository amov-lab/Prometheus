#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>

#define snap(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

typedef struct sdf_cell {
    double dx;
    double dy;
} sdf_cell_t;

class ImageSDF
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber binary_sub_;
    image_transport::Publisher sdf_preview_pub_;
    image_transport::Publisher sdf_raw_pub_;
    std::vector< std::vector<double> > distance_field_;
    double max_distance_;
    double min_distance_;

public:

    ImageSDF(ros::NodeHandle &n, std::string binary_base_topic, std::string sdf_preview_topic, std::string sdf_raw_topic) : nh_(n), it_(n)
    {
        max_distance_ = 0.0;
        min_distance_ = 0.0;
        binary_sub_ = it_.subscribe(binary_base_topic, 1, &ImageSDF::camera_cb, this);
        sdf_preview_pub_ = it_.advertise(sdf_preview_topic, 1, true);
        sdf_raw_pub_ = it_.advertise(sdf_raw_topic, 1, true);
        std::string transport_in = binary_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    void loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    void update_sdf(cv::Mat& image)
    {
        ROS_DEBUG("Making intermediate containers for SDF");
        std::vector< std::vector<sdf_cell_t> > empty_cells;
        std::vector< std::vector<sdf_cell_t> > filled_cells;
        // Resize the fields
        int width = image.cols;
        int height = image.rows;
        empty_cells.resize(height);
        filled_cells.resize(height);
        distance_field_.resize(height);
        for (int i = 0; i < height; i++)
        {
            empty_cells[i].resize(width);
            filled_cells[i].resize(width);
            distance_field_[i].resize(width);
        }
        ROS_DEBUG("Marking filled/empty pixels for SDF");
        // Go through the image and add filled/empty pixels to the corresponding fields
        sdf_cell_t empty_cell;
        empty_cell.dx = INFINITY;
        empty_cell.dy = INFINITY;
        sdf_cell_t filled_cell;
        filled_cell.dx = 0.0;
        filled_cell.dy = 0.0;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                uint8_t pixel = image.at<uint8_t>(i,j);
                if (pixel != 0)
                {
                    filled_cells[i][j] = filled_cell;
                    empty_cells[i][j] = empty_cell;
                }
                else
                {
                    filled_cells[i][j] = empty_cell;
                    empty_cells[i][j] = filled_cell;
                }
            }
        }
        ROS_DEBUG("Running 8SSEDT on intermediate containers");
        // Run the 8SSEDT algorithm to compute the partial SDFs
        update_partial_sdf(filled_cells);
        update_partial_sdf(empty_cells);
        ROS_DEBUG("Computing the final SDF");
        // Combine the partial fields to form the SDF
        max_distance_ = 0.0;
        min_distance_ = 0.0;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                double filled_distance = sqrt(pow(filled_cells[i][j].dx, 2) + pow(filled_cells[i][j].dy, 2));
                double empty_distance = sqrt(pow(empty_cells[i][j].dx, 2) + pow(empty_cells[i][j].dy, 2));
                double new_distance = filled_distance - empty_distance;
                distance_field_[i][j] = new_distance;
                if (new_distance > max_distance_)
                {
                    max_distance_ = new_distance;
                }
                if (new_distance < min_distance_)
                {
                    min_distance_ = new_distance;
                }
            }
        }
    }

    bool update_partial_sdf(std::vector< std::vector<sdf_cell_t> >& partial_field)
    {
        if (partial_field.size() > 0)
        {
            // Pass 1.1.0
            for (size_t y = 0; y < partial_field[0].size(); y++)
            {
                // Pass 1.1.1
                for (size_t x = 0; x < partial_field.size(); x++)
                {
                    // Get value from grid
                    sdf_cell_t cell = get(partial_field, x, y);
                    // Do work
                    compare(partial_field, cell, x, y, -1, 0);
                    compare(partial_field, cell, x, y, 0, -1);
                    compare(partial_field, cell, x, y, -1, -1);
                    compare(partial_field, cell, x, y, 1, -1);
                    // Store back into grid
                    put(partial_field, cell, x, y);
                }
                // Pass 1.1.2
                for (int x = (partial_field.size() - 1); x >= 0; x--)
                {
                    // Get value from grid
                    sdf_cell_t cell = get(partial_field, x, y);
                    // Do work
                    compare(partial_field, cell, x, y, 1, 0);
                    // Store back into grid
                    put(partial_field, cell, x, y);
                }
            }
            // Pass 1.2.0
            for (int y = (partial_field[0].size() - 1); y >= 0; y--)
            {
                // Pass 1.1.1
                for (int x = (partial_field.size() - 1); x >= 0; x--)
                {
                    // Get value from grid
                    sdf_cell_t cell = get(partial_field, x, y);
                    // Do work
                    compare(partial_field, cell, x, y, 1, 0);
                    compare(partial_field, cell, x, y, 0, 1);
                    compare(partial_field, cell, x, y, -1, 1);
                    compare(partial_field, cell, x, y, 1, 1);
                    // Store back into grid
                    put(partial_field, cell, x, y);
                }
                // Pass 1.1.2
                for (size_t x = 0; x < partial_field.size(); x++)
                {
                    // Get value from grid
                    sdf_cell_t cell = get(partial_field, x, y);
                    // Do work
                    compare(partial_field, cell, x, y, -1, 0);
                    // Store back into grid
                    put(partial_field, cell, x, y);
                }
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    inline sdf_cell_t get(std::vector< std::vector<sdf_cell_t> >& partial_field, size_t x, size_t y)
    {
        if (x < partial_field.size())
        {
            if (y < partial_field[x].size())
            {
                return partial_field[x][y];
            }
        }
        sdf_cell_t empty;
        empty.dx = INFINITY;
        empty.dy = INFINITY;
        return empty;
    }

    inline void put(std::vector< std::vector<sdf_cell_t> >& partial_field, sdf_cell_t& cell, size_t x, size_t y)
    {
        if (x < partial_field.size())
        {
            if (y < partial_field[x].size())
            {
                partial_field[x][y] = cell;
            }
        }
    }

    inline double distance_squared(sdf_cell_t& cell)
    {
        return ((cell.dx * cell.dx) + (cell.dy * cell.dy));
    }

    inline void compare(std::vector< std::vector<sdf_cell_t> >& partial_field, sdf_cell_t& cell, int x, int y, int x_offset, int y_offset)
    {
        sdf_cell_t other = get(partial_field, x + x_offset, y + y_offset);
        other.dx += x_offset;
        other.dy += y_offset;
        if (distance_squared(other) < distance_squared(cell))
        {
            cell = other;
        }
    }

    void camera_cb(const sensor_msgs::ImageConstPtr& image)
    {
        ROS_DEBUG("Got new image to resize and SDF");
        // Convert to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Make destination
        cv::Mat binary; //(cv::Size(resized_width_, resized_height_), CV_8UC1);
        binary = cv_ptr->image;
        // Compute the SDF for the image
        ROS_DEBUG("Attempting to compute SDF of image...");
        update_sdf(binary);
        ROS_DEBUG("...SDF compute finished");
        // Publish the raw SDF
        cv::Mat raw_sdf_image(cv::Size(binary.cols, binary.rows), CV_32FC2);
        for (int i = 0; i < binary.rows; i++)
        {
            for (int j = 0; j < binary.cols; j++)
            {
                double current_distance = distance_field_[i][j];
                if (current_distance >= 0.0)
                {
                    raw_sdf_image.at<cv::Vec2f>(i,j)[0] = fabs(current_distance);
                    raw_sdf_image.at<cv::Vec2f>(i,j)[1] = 0.0;
                }
                else
                {
                    raw_sdf_image.at<cv::Vec2f>(i,j)[0] = 0.0;
                    raw_sdf_image.at<cv::Vec2f>(i,j)[1] = fabs(current_distance);
                }
            }
        }
        // Convert back to ROS
        sensor_msgs::Image sdf_raw_image;
        cv_bridge::CvImage sdf_raw_converted(image->header, sensor_msgs::image_encodings::TYPE_32FC2, raw_sdf_image);
        sdf_raw_converted.toImageMsg(sdf_raw_image);
        // Republish
        sdf_raw_pub_.publish(sdf_raw_image);
        // Convert SDF to false-color image
        cv::Mat false_color_sdf(cv::Size(binary.cols, binary.rows), CV_8UC3);
        for (int i = 0; i < binary.rows; i++)
        {
            for (int j = 0; j < binary.cols; j++)
            {
                double current_distance = distance_field_[i][j];
                uint8_t blue_channel = 0x00;
                uint8_t red_channel = 0x00;
                uint8_t green_channel = 0x00;
                if (current_distance > 0.0)
                {
                    red_channel = (uint8_t)(64.0 + (64.0 * fabs(current_distance / max_distance_)));
                }
                else if (current_distance == 0.0 || current_distance == -0.0)
                {
                    green_channel = 0xff;
                }
                else
                {
                    blue_channel = (uint8_t)(64.0 + (64.0 * fabs(current_distance / min_distance_)));
                }
                false_color_sdf.at<cv::Vec3b>(i, j)[0] = blue_channel;
                false_color_sdf.at<cv::Vec3b>(i, j)[1] = green_channel;
                false_color_sdf.at<cv::Vec3b>(i, j)[2] = red_channel;
            }
        }
        // Convert back to ROS
        sensor_msgs::Image sdf_preview_image;
        cv_bridge::CvImage sdf_preview_converted(image->header, sensor_msgs::image_encodings::BGR8, false_color_sdf);
        sdf_preview_converted.toImageMsg(sdf_preview_image);
        // Republish
        sdf_preview_pub_.publish(sdf_preview_image);
        ROS_DEBUG("Resize + SDF finished");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_sdf");
    ROS_INFO("Starting SDF from image generator...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string binary_base_topic;
    std::string sdf_preview_topic;
    std::string sdf_raw_topic;
    nhp.param(std::string("binary_base_topic"), binary_base_topic, std::string("camera/rgb/binary"));
    nhp.param(std::string("sdf_preview_topic"), sdf_preview_topic, std::string("camera/rgb/sdf"));
    nhp.param(std::string("sdf_raw_topic"), sdf_raw_topic, std::string("camera/rgb/sdf_raw"));
    ImageSDF processor(nh, binary_base_topic, sdf_preview_topic, sdf_raw_topic);
    ROS_INFO("...startup complete");
    processor.loop();
    return 0;
}
