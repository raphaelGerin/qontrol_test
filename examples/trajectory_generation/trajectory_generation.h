#include <Eigen/Geometry>

// Create a trajectory class with isometry_array and duration as private members
class TrajectoryGeneration
{
public:
    TrajectoryGeneration(const std::string& filename, double time_step)
    {
        time_step_ = time_step;
        readTrajectoryFromFile(filename);
    }

    // read the trajectory from a csv file
    void readTrajectoryFromFile(const std::string& filename)
    {  
        std::ifstream file(filename);
        if (file.is_open())
        {
            std::string line;
            while (std::getline(file, line))
            {
                std::istringstream iss(line);
                // ignore first line
                if (line.find("x,y,z,qx,qy,qz,qw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az,aroll,apitch,ayaw") != std::string::npos)
                {
                    continue;
                }
                std::string token;
                std::vector<std::string> tokens;
                while (std::getline(iss, token, ','))
                {
                    tokens.push_back(token);
                }
                Eigen::Vector3d position(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2]));
                Eigen::Quaternion quat(std::stod(tokens[6]), std::stod(tokens[3]), std::stod(tokens[4]), std::stod(tokens[5]));
                isometry_array.push_back(constructIsometryFromPose(position, quat));
                // Get the velocity twist as the 6 next token in the line and file an Eigen::Matrix<double, 6, 1>
                // Initialise a Eigen Matrix<double, 6, 1> with the 6 next token in the line

                Eigen::Matrix<double, 6, 1> velocity;
                velocity << std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]), std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]);

                velocity_twist.push_back(velocity) ;
                Eigen::Matrix<double, 6, 1> acceleration;
                acceleration << std::stod(tokens[12]), std::stod(tokens[13]), std::stod(tokens[14]), std::stod(tokens[15]), std::stod(tokens[16]), std::stod(tokens[17]);
                // Get the acceleration twist as the 6 next token in the line
                acceleration_twist.push_back(acceleration);
            }
            duration = isometry_array.size() * time_step_;
        }
    }

    // Fetch the next pose of the end-effector in the isometry array for a given time
    // If the time is greater than the duration of the trajectory, return the last pose
    void update()
    {
        time_ += time_step_;
        if (time_ > duration)
        {
            pose = isometry_array.back();
            velocity = velocity_twist.back();
            acceleration = acceleration_twist.back();
        }
        else
        {
            int index = time_ / duration * isometry_array.size();
            pose = isometry_array[index];
            velocity = velocity_twist[index];
            acceleration = acceleration_twist[index];
        }
    }
    
    // Construct an isometry from a position and Eigen::Quaternion
    Eigen::Isometry3d constructIsometryFromPose(const Eigen::Vector3d& position, Eigen::Quaterniond quat)
    {
        Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
        isometry.translation() = position;
        isometry.linear() = quat.toRotationMatrix();
        return isometry;
    }


    private:
    std::vector<Eigen::Isometry3d> isometry_array;
    std::vector<Eigen::Matrix<double, 6, 1>> velocity_twist;
    std::vector<Eigen::Matrix<double, 6, 1>> acceleration_twist;
    double duration = 0.0;
    public:
    Eigen::Isometry3d pose;
    Eigen::Matrix<double, 6, 1> velocity;
    Eigen::Matrix<double, 6, 1> acceleration;
    double time_step_ = 0.001;
    double time_ = 0;
};