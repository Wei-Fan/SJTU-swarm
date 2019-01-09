#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    // arguments: executable, path_to_config_dir, path_to_generated_launch_file, control_yaml_file

    string trialConfigPath = argv[1];
    string launchGenerationPath = argv[2];

    cout << "ADDING CONFIG FROM" << trialConfigPath << " TO " << launchGenerationPath << endl;
    SwarmGenerator s(trialConfigPath, launchGenerationPath);

    //now take triplets containing the trial name, yaml file name, and # times to run.

    if(!s.addTrial(argv[3])) {
        cerr << "ERROR ADDING CONFIG: " << argv[3] << " check to ensure it is formatted correctly and exists." << endl;
        return 0;
    }

    s.runTrials();

    return 0;
}