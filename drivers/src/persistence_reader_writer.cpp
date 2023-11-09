#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>

#include "classes/persistence/persistence_reader_writer.h"


PersistenceReaderWriter&
PersistenceReaderWriter::GetInstance(void)
{
    static PersistenceReaderWriter instance;

    return instance;
}


void
PersistenceReaderWriter::Initialize(std::string ros_pkg_name)
{
    if(!json_initialized_)
    {
        ros_pkg_name_ = ros_pkg_name;
        CreatePersistenceFolderIfNotPresent();
        CreatePersistenceFileIfNotPresentOrInvalid();
        FillJsonFromFile();
        json_initialized_ = true;
    }
}


int
PersistenceReaderWriter::ReadInt(std::string key)
{
    int out_int = 0;

    Initialize(ros_pkg_name_);

    if(persitence_json_.contains(key)
       && ((persitence_json_[key].type()
            == nlohmann::json::value_t::number_integer)
           || (persitence_json_[key].type()
               == nlohmann::json::value_t::number_unsigned)))
    {
        out_int = persitence_json_[key].get<int>();
    }
    else
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The key \"" << key << "\" for int value couldn't be"
                       << " found in the persistent data.";

        throw std::invalid_argument(err_msg_stream.str());
    }

    return out_int;
}


void
PersistenceReaderWriter::WriteInt(std::string key, int value)
{
    Initialize(ros_pkg_name_);

    persitence_json_[key] = value;
}


void
PersistenceReaderWriter::Dump(void)
{
    Initialize(ros_pkg_name_);

    {
        std::string file_path = GetPersistentFilePath();
        std::ofstream file_out_stream(file_path);

        file_out_stream << persitence_json_ << std::endl;
    }
}


PersistenceReaderWriter::PersistenceReaderWriter(void)
{
}


void
PersistenceReaderWriter::CreatePersistenceFolderIfNotPresent(void)
{
    std::string folder_path = GetPersistentFolderPath();

    if(!FolderExists(folder_path))
    {
        CreateFolder(folder_path);
    }
}


void
PersistenceReaderWriter::CreatePersistenceFileIfNotPresentOrInvalid(void)
{
    std::string file_path = GetPersistentFilePath();

    if(!FileExists(file_path) || !JsonFileValid(file_path))
    {
        CreateEmptyJsonFile(file_path);
    }
}


void
PersistenceReaderWriter::FillJsonFromFile(void)
{
    std::string json_file_path = GetPersistentFilePath();
    std::ifstream file_stream(json_file_path);

    persitence_json_ = nlohmann::json::parse(file_stream);
}


std::string
PersistenceReaderWriter::GetPersistentFolderPath(void)
{
    std::stringstream folder_path_stream;

    folder_path_stream << GetHomePath() << "/" << kPersistentDataFolder << "/"
                       << ros_pkg_name_;

    return folder_path_stream.str();
}


std::string
PersistenceReaderWriter::GetPersistentFilePath(void)
{
    std::stringstream file_path_stream;

    file_path_stream << GetPersistentFolderPath() << "/" << GetNodeName()
                     << kJsonExtension;

    return file_path_stream.str();
}


std::string
PersistenceReaderWriter::GetNodeName(void)
{
    return ros::this_node::getName();
}


std::string
PersistenceReaderWriter::GetHomePath(void)
{
    return getenv(kHomeEnvVar);
}


bool
PersistenceReaderWriter::FolderExists(std::string folder_path)
{
    struct stat file_info;

    return ((stat(folder_path.c_str(), &file_info) == 0)
            && (file_info.st_mode & S_IFDIR));
}


bool
PersistenceReaderWriter::FileExists(std::string file_path)
{
    struct stat file_info;

    return ((stat(file_path.c_str(), &file_info) == 0)
            && !(file_info.st_mode & S_IFDIR));
}


bool
PersistenceReaderWriter::JsonFileValid(std::string json_file_path)
{
    std::ifstream file_stream(json_file_path);

    return nlohmann::json::accept(file_stream);
}


void
PersistenceReaderWriter::CreateFolder(std::string folder_path)
{
    std::stringstream cmd_stream;
    int error_code = 0;

    cmd_stream << kMkdirCmd << " " << folder_path;

    error_code = system(cmd_stream.str().c_str());
    if(error_code != 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The command \"" << cmd_stream.str() << "\" failed"
                       << " while trying to create a folder.";

        throw std::runtime_error(err_msg_stream.str());
    }
}


void
PersistenceReaderWriter::CreateEmptyJsonFile(std::string file_path)
{
    std::stringstream cmd_stream;
    int error_code = 0;

    cmd_stream << kEmptyJsonCmd << " " << file_path;

    error_code = system(cmd_stream.str().c_str());
    if(error_code != 0)
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The command \"" << cmd_stream.str() << "\" failed"
                       << " while trying to create a file.";

        throw std::runtime_error(err_msg_stream.str());
    }
}
