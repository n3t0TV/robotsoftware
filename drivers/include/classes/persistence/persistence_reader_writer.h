#pragma once

#include <string>

#include "libraries/json.hpp"


class PersistenceReaderWriter
{
    public:
        static PersistenceReaderWriter& GetInstance(void);

        void Initialize(std::string ros_pkg_name);

        int ReadInt(std::string key);

        void WriteInt(std::string key, int value);

        void Dump(void);

    private:
        PersistenceReaderWriter(void);

        /* Don't implement this constructor and keep it inaccessible to avoid
         * copies of the singleton */
        PersistenceReaderWriter(PersistenceReaderWriter const&);

        /* Don't implement this function and keep it inaccessible to avoid
         * copies of the singleton */
        void operator=(PersistenceReaderWriter const&);

    private:
        const char* kPersistentDataFolder = ".brain_persistent_data";

        const char* kMkdirCmd = "mkdir -p";

        const char* kEmptyJsonCmd = "echo \"{}\" >";

        const char* kJsonExtension = ".json";

        const char* kHomeEnvVar = "HOME";

        nlohmann::json persitence_json_;

        bool json_initialized_ = false;

        std::string ros_pkg_name_;

        void CreatePersistenceFolderIfNotPresent(void);

        void CreatePersistenceFileIfNotPresentOrInvalid(void);

        void FillJsonFromFile(void);

        std::string GetPersistentFolderPath(void);

        std::string GetPersistentFilePath(void);

        std::string GetNodeName(void);

        std::string GetHomePath(void);

        bool FolderExists(std::string folder_path);

        bool FileExists(std::string file_path);

        bool JsonFileValid(std::string json_file_path);

        void CreateFolder(std::string folder_path);

        void CreateEmptyJsonFile(std::string file_path);
};
