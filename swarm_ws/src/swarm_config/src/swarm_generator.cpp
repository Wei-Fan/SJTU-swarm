#include <deque>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <ros/package.h>
#include <fstream>
#include <unistd.h>
#include <vector>
#include <bits/stdc++.h>
#include "yaml-cpp/yaml.h"

using namespace std;

class SwarmGenerator {
private:
    struct TRIAL {
        std::string trialConfigFile = "";
//        int numTimesToRunTrial = 0;
//        std::string trialName = "";

        /*if a folder with given trial name already exists, this offset will be incremented to ensure a unique folder is created
         * (containing this offset). If offset = -1, then it will not be included */
        int offset = -1;
    };
    typedef struct TRIAL Trial;

    /**
     * List of the PIDS of active trials.
     */
    std::vector<int> runningTrials;

    std::string DEFAULT_BEHAVIOUR_LIB_FILE = "default_behaviours";
//    std::string DEFAULT_MAP_NAME = "big_arena.world";
    double DEFAULT_CONNECTIVITY_RADIUS = 10.0;

    /**
     * The trials that are pending to be run.
     */
//    std::deque<Trial*> trialBacklog;

    string trialConfigPath = "";
    string launchGenerationPath = "";

    bool trialFileExists(const std::string &fileName) const{
        string path = this->trialConfigPath + fileName;
        ifstream file(path.c_str());
        return file.good();
    }

    //check if a folder for given trial has been generated already.
    bool trialDirExists(const std::string &path) const{
        struct stat status;
        return stat(path.c_str(), &status) == 0;
    }

    void launchTrial(Trial &t){
        t.numTimesToRunTrial--; //we update this only in the original parent process...
        cout << this->launchGenerationPath << "\n";
        string trialLaunchFile = getTrialLaunchPath(t) + to_string(t.numTimesToRunTrial) + ".launch";
        int pid = fork();

        if(pid==0) { /* child */
            system(("roslaunch " +  trialLaunchFile).c_str());
            exit(0);
        } else { /* parent */
            this->runningTrials.push_back(pid);
        }
    }

//    std::string getTrialLaunchPath(const Trial &t){
//        if(this->launchGenerationPath.size()==0) {
//            if (t.offset >= 0) {
//                return ros::package::getPath("cmuswarm_performance") + "/launch/trials/" + t.trialName +
//                       to_string(t.offset) +
//                       "/";
//            } else {
//                return ros::package::getPath("cmuswarm_performance") + "/launch/trials/" + t.trialName + "/";
//            }
//        } else {
//            if (t.offset >= 0) {
//                return this->launchGenerationPath + t.trialName + "_" +
//                       to_string(t.offset) +
//                       "/";
//            } else {
//                return this->launchGenerationPath + t.trialName + "_0/";
//            }
//        }
//    }

    /**
     * Whether or not a trial is running. This is temporary as it only enables 1 trial to run at a time... eventually we wish to have parallelism with multiple trials running.
     */
    bool active = false;
public:
    SwarmGenerator(){
        this->trialConfigPath = ros::package::getPath("swarm_config")+"/src/";
        this->launchGenerationPath = ros::package::getPath("swarm_config") + "/launch/";

    }

    SwarmGenerator(const std::string &trialConfigPath, const std::string &launchGenerationPath){
        this->trialConfigPath = trialConfigPath;
        this->launchGenerationPath = launchGenerationPath;
//        this->trialBacklog = {};

        cout << "Constructed with " << this->launchGenerationPath << " and " << this->trialConfigPath << endl;

    }

    /**
     * Add a trial to backlog, and have it run numTimes.
     * @param trialConfigFile: The trial config file. This will be used to generate trials.
     * @param trialName: A unique name for the trial suite. Generated launch files/performance results will have this name.
     * @param numTimes: How many times to run this experimental trial.
     * @return: True if trial was added. False if not.
     */
    bool addTrial(const std::string &configFile){
        if (this->trialFileExists(configFile)) {
            Trial *t = new Trial;
            t->trialConfigFile = configFile;
//            t->numTimesToRunTrial = numTimes;
//            t->trialName = trialName;

            //open the trial config file
            std::string trialConfigFile = this->trialConfigPath + t->trialConfigFile;
            try {
                YAML::Node config = YAML::LoadFile(trialConfigFile);

                /*Refer to YAML file*/
//                int xLow = config["robot_spawn_region"][0][0].as<int>();
//                int xHigh = config["robot_spawn_region"][0][1].as<int>();
//                int yLow = config["robot_spawn_region"][1][0].as<int>();
//                int yHigh = config["robot_spawn_region"][1][1].as<int>();

                /*behavior*/
                string behaviourLib = this->DEFAULT_BEHAVIOUR_LIB_FILE;
                if(config["behaviour_lib"]) {
                    behaviourLib = config["behaviour_lib"].as<string>();
                }

//                string map_name = this->DEFAULT_MAP_NAME;
//                if(config["map"]){
//                    map_name = config["map"].as<string>();
//                }

                /*connectivity radius*/
                double connectivity_radius = this->DEFAULT_CONNECTIVITY_RADIUS;
                if(config["connectivity_radius"]){
                    connectivity_radius = config["connectivity_radius"].as<double>();
                }

                /*read subswarm types and number of robots*/
                unordered_map<string, int> robotSwarm;
                if(config["robot_swarm"]){
                    for(auto const &node : config["robot_swarm"]){
                        pair<string, int> subswarm;
                        subswarm.first = node["robot_type"].as<string>();
                        subswarm.second = node["num_robots"].as<int>();
                        robotSwarm.insert(subswarm);
                    }
                }
                else{
                    pair<string, int> defaultSwarm;
                    defaultSwarm.first = "ugv";
                    defaultSwarm.second = config["num_robots"].as<int>();
                    robotSwarm.insert(defaultSwarm);
                }

//                TrialFileGenerator gen;
                /*behavior request*/
//                for(auto const &node : config["behaviour_requests"]) {
//                    const std::string behaviourName = node["name"].as<std::string>();
//                    int time = node["time"].as<int>();
//
//                    //get int params
//                    vector<pair<string, int>> int_params;
//
//                    if(node["ints"]) {
//                        for (const auto &it : node["ints"]) {
//                            int_params.push_back(
//                                    std::make_pair<const string, int>(it["name"].as<string>(), it["value"].as<int>()));
//                        }
//                    }
//
//                    //get double params
//                    vector<pair<string, double>> double_params;
//
//                    if(node["doubles"]) {
//                        for (const auto &it : node["doubles"]) {
//                            double_params.push_back(
//                                    std::make_pair<const string, double>(it["name"].as<string>(), it["value"].as<double>()));
//                        }
//                    }
//
//                    //get string params
//                    vector<pair<string, string>> str_params;
//
//                    if(node["strs"]) {
//                        for (const auto &it : node["strs"]) {
//                            str_params.push_back(
//                                    std::make_pair<const string, const string>(it["name"].as<string>(), it["value"].as<string>()));
//                        }
//                    }
//
//                    //get bool
//                    vector<pair<string, bool>> bool_params;
//                    if(node["bools"]) {
//                        for (const auto &it : node["bools"]) {
//                            bool_params.push_back(
//                                    std::make_pair<const string, bool>(it["name"].as<string>(), it["value"].as<bool>()));
//                        }
//                    }
//
//                    gen.addBehaviourRequest(behaviourName, time, int_params, str_params, double_params, bool_params);
//                }

                /*collect performance metrics and termination conditions*/
//                unordered_map<string, vector<pair<string, double>>> performance_metrics;
//
//                for(auto const &metric : config["performance_summarizers"]) {
//                    string metricName = metric["metric_name"].as<string>();
//                    vector<pair<string, double>> terminationConditions;
//
//                    for(auto const &condition : metric["termination_conditions"]) {
//                        terminationConditions.push_back(make_pair<const string, double>(condition["name"].as<string>(), condition["value"].as<double>()));
//                    }
//
//                    performance_metrics[metricName] = terminationConditions;
//                }

//                vector<pair<string, string>> loggers;
//                for(auto const &logger : config["topic_loggers"]) {
//                    string loggerName = logger["logger_name"].as<string>();
//                    string expression = logger["topic_expr"].as<string>();
//                    loggers.push_back(make_pair<const string, const string>(logger["logger_name"].as<string>(), logger["topic_expr"].as<string>()));
//                }

                /*generating a launch file*/
                TrialFileGenerator gen;
                gen.setBehaviourLibrary(behaviourLib);
                gen.setFileNamePrefix("");
//                gen.setTrialName(trialName);
                gen.setNumRobots(robotSwarm);
//                gen.setSpawnRegionX(xLow, xHigh);
//                gen.setSpawnRegionY(yLow, yHigh);
//                gen.setPerformanceMetrics(performance_metrics);
//                gen.setLoggers(loggers);
                gen.setGenerationPath(this->launchGenerationPath);//this->getTrialLaunchPath(*t));
//                gen.setSeedPath(ros::package::getPath("cmuswarm_swarm")+"/world/" + map_name);
                gen.setConnectivityRadius(connectivity_radius);

//                std::string launchGenPath = this->getTrialLaunchPath(*t);
//                if (this->trialDirExists(launchGenPath)) {
//                    int offsetIndex = 0;
//                    std::cout << "Already exists path: " << launchGenPath << "\n";
//                    launchGenPath.erase(launchGenPath.size()-2, 2);
//                    std::cout << "erased path: " << launchGenPath << "\n";
//                    while (this->trialDirExists((launchGenPath + to_string(offsetIndex) + "/"))) {
//                        cout << "Directory " << launchGenPath + to_string(offsetIndex) +
//                                                "/ already exists. Trying a new unique identifier.\n";
//                        offsetIndex++;
//                    }
//                    std::cout << "offset index " << offsetIndex << "\n";
//                    t->offset = offsetIndex;
//
//                    launchGenPath = launchGenPath + to_string(offsetIndex)+"/";
//
//                    gen.generate(t->numTimesToRunTrial,
//                                 launchGenPath);
//                } else {
//                    gen.generate(t->numTimesToRunTrial,
//                                 launchGenPath);
//                }
                gen.generate(this->launchGenerationPath);


//                this->trialBacklog.push_back(t);
                return true;
            } catch (exception e) {
                cout << "Exception while configuring trial: " << e.what() << ", ensure the config is formatted correctly!" << endl;
                return false;
            }
        } else {
            cout << "Unable to add trial config file (" << configFile << ") does not exist." << endl;
            return false;
        }
    }

    /**
     * Check which trial processes have terminated (Ie their process id doesn't exist anymore) and then deque them so we can run more trials.
     */
    void cleanCompletedTrials();

    /**
     * Launch trials that are available to be run. That is they have threads to be run on, and are in the backlog.
     */
    void runTrials();
};
