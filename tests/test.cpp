#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

using namespace std;

// Function to read the second column (Cl) from the given file
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

int main() {
    const string input_file = "input.json";
    const string output_file = "output_files/cl_cd_pitch_plunge_k=1.2_n=101.dat";
    const string reference_file = "tests/cl_cd_pitch_plunge_k=1.2_n=101_ref.dat";
    const double tolerance = 1e-2;

    // Run the main solver
   int return_code = system(("./PANKH_solver " + input_file).c_str());
    if (return_code != 0) {
        cerr << "Solver execution failed with code " << return_code << endl;
        return 1;
    }

    // Read solver output and reference data
    vector<double> output_cl = read_Cl_column(output_file);
    vector<double> reference_cl = read_Cl_column(reference_file);

    // Check for matching size
    if (output_cl.size() != reference_cl.size()) {
        cerr << "Mismatch in number of time steps: "
             << "output has " << output_cl.size()
             << ", reference has " << reference_cl.size() << endl;
        return 1;
    }

    // Compare values
    bool pass = true;
    for (size_t i = 0; i < output_cl.size(); ++i) {
        double diff = fabs(output_cl[i] - reference_cl[i]);
        if (diff > tolerance) {
            cerr << "Mismatch at timestep " << i << ": "
                 << "output = " << output_cl[i]
                 << ", reference = " << reference_cl[i]
                 << ", |diff| = " << diff << endl;
            pass = false;
        }
    }

    if (pass) {
        cout << "✅ Test Passed: All Cl values within tolerance of " << tolerance << endl;
        return 0;
    } else {
        cerr << "❌ Test Failed: Some Cl values exceeded tolerance." << endl;
        return 1;
    }
}
