#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <cstdlib>

using namespace std;

// Function to read Cl column from a file
vector<double> read_Cl_column(const string& filename) {
    vector<double> cl_values;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        exit(1);
    }

    double time, cl, cd;
    while (file >> time >> cl >> cd) {
        cl_values.push_back(cl);
    }
    return cl_values;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <input_json> [reference_file]" << endl;
        return 1;
    }

    string input_file = argv[1];
    string output_file = "output_files/cl_cd_pitch_plunge_k=1.2_n=101.dat";
    string reference_file = (argc >= 3) ? argv[2] : "tests/cl_cd_pitch_plunge_k=1.2_n=101_ref.dat";

    // Run the solver
    int ret = system(("./PANKH_solver " + input_file).c_str());
    if (ret != 0) {
        cerr << "Solver execution failed with code " << ret << endl;
        return 1;
    }

    // Read output and reference
    vector<double> output_cl = read_Cl_column(output_file);
    vector<double> reference_cl = read_Cl_column(reference_file);

    if (output_cl.size() != reference_cl.size()) {
        cerr << "Mismatch in number of time steps: output = " 
             << output_cl.size() << ", reference = " << reference_cl.size() << endl;
        return 1;
    }

    const double tolerance = 1e-2;
    bool pass = true;
    for (size_t i = 0; i < output_cl.size(); ++i) {
        double diff = fabs(output_cl[i] - reference_cl[i]);
        if (diff > tolerance) {
            cerr << "Mismatch at timestep " << i 
                 << ": output = " << output_cl[i] 
                 << ", reference = " << reference_cl[i] 
                 << ", |diff| = " << diff << endl;
            pass = false;
        }
    }

    if (pass) {
        cout << "Test Passed: All Cl values within the prescribed tolerance" << endl;
        return 0;
    } else {
        cerr << "Test Failed: Some Cl values exceeded tolerance." << endl;
        return 1;
    }
}
