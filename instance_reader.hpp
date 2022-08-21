//------------------------------------------------------------------------------------------------------------//
/* Read instance data from csv file.*/
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
typedef vector<size_t> VecT;
//------------------------------------------------------------------------------------------------------------//
// Struct definitions
struct Model{
    int Id; // model identifier
    int Workload; // total workload for model
    int Min_sct; // lower bound for segment cycle time for model
    VecB Tasks_per_model; // list of required tasks for model
    VecI Task_times; // list of task times for model
    vector<VecB> Successors; // explicit successors: 1 if t2 has to be performed after t1
    vector<VecB> All_successors; // implicit successors: 1 if t2 has to be performed after t1
    vector<VecB> All_predecessors; // implicit predecessors: 1 if t2 has to be performed before t1
    vector<VecB> Preselect_assignments; // task-location assignments for NF and OF scenarios: 1 if task is assigned to location
    vector<VecB> Preselect_all_successors; // implicit successors in NF and RF scenarios: 1 if task t2 has to be after before t1
    vector<VecB> Preselect_all_predecessors; // implicit predecessors in NF and RF scenarios: 1 if task t2 has to be performed before t1
    vector<vector<VecB>> Possible_task_location_combinations; // possible task location combinations for model
    vector<vector<VecI>> Possible_task_location_combinations_list; // possible task location combinations for model (as task lists)
    vector<VecI> Possible_task_location_combination_workloads; //possible task location combination workloads for model
    vector<VecI> Preselect_route; // route that is preselected in scenario NF for model
    VecI Preselect_route_workloads; // workloads on preselected route in scenario NF for model
    vector<VecI> Scenario_task_location_combinations; // available task-location-combintations for model in respective scenario
    vector<VecB> Scenario_all_successors; // implicit successors in given scenario: 1 if t2 has to be performed after t1
    vector<VecB> Scenario_all_predecessors; // implicit predecessors in given scenario: 1 if t2 has to be performed before t1
};
struct Vehicle{
    int Id; // vehicle identifier
    int Arrival_time; // vehicle arrival time
    int Type; // model type
    VecI All_routes; // all routes for vehicle
    VecI Available_routes; // available routes for vehicle
    vector<vector<VecI>> Route_details; // route information, i.e., sequence of (l,c,z) tuples
    VecI Route_scts; // minimum required layout cycle time for routes
    vector<VecI> Route_start_times; // start times of routes at locations
    vector<VecI> Route_end_times; // end times of routes at locations
    vector<VecI> Available_task_location_combinations; // task location combinations available for vehicle
    double Max_cost; // dual cost from route selection constraints
    double Sct_cost; // dual cost from sct constraints
    double Current_sct; // current sct in relaxed master problem
};
struct Location{
    int Id; // location identifier
    int Level_index; // level index of location
    int Row_index; //row index of location
    VecB Assignments; // task assignments: 1 if task can be performed at location
};
//------------------------------------------------------------------------------------------------------------//
/* A class to read data from a csv file. */
class CSVReader
{
    string fileName;
    string delimeter;
public:
    CSVReader(string filename, string delm = ",") :
            fileName(filename), delimeter(delm)
    { }
    // function to fetch data from a CSV File
    vector<VecS> get_data();
};
/* Parses through csv file line by line and returns the data in vector of vector of strings. */
vector<VecS> CSVReader::get_data() {
    ifstream file(fileName);
    if (!file.good()){
        cout << "WARNING: File " << fileName << " does not exist!" << endl;
        throw runtime_error("END");
    }
    vector<VecS> dataList;
    string line = "";
    // iterate through each line and split the content using delimeter
    while (getline(file, line))	{
        VecS vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        dataList.push_back(vec);
    }
    // close the File
    file.close();
    return dataList;
}
//------------------------------------------------------------------------------------------------------------//
/* Function to read csv data from file for given instance. */
void read_instance(string &filename, int &number_models, int &number_vehicles, int &number_locations, int &number_tasks,
        vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations,
        int &cycle_time, Location &start_loc, Location &end_loc, vector<VecI> &transportation_times, int &total_workload) {
    // creating an object of CSVReader
    string filename_with_path("Data/Instances_csv/" + filename);
    CSVReader reader(filename_with_path, ";");
    // get the data from CSV File
    vector<VecS> data = reader.get_data();
    // read instance size
    number_models = stoi(data[1][2]);
    number_locations = stoi(data[1][0]);
    number_tasks = stoi(data[1][1]);
    // read general data
    cycle_time = stoi(data[3][0]);
    total_workload = stoi(data[3][1]);
    int start_loc_id = stoi(data[9][0]);
    int end_loc_id = stoi(data[9][1]);
    // define models
    for (int m = 0; m != number_models; ++m) {
        Model mod;
        mod.Id = m;
        models.push_back(mod);
    }
    // define vehicles
    for (int v = 0; v != number_vehicles; ++v) {
        Vehicle veh;
        veh.Id = v;
        veh.Arrival_time = stoi(data[7][v]);
        veh.Type = stoi(data[5][v]);
        vehicles.push_back(veh);
    }
    // define locations
    int row_count(11);
    for (int l = 0; l != number_locations; ++l, ++row_count) {
        Location loc;
        loc.Id = l;
        loc.Level_index = stoi(data[row_count][0]);
        loc.Row_index = stoi(data[row_count][1]);
        locations.push_back(loc);
    }
    ++row_count;
    // define transportation times
    for (int l1 = 0; l1 != number_locations; ++l1, ++row_count){
        VecI tt;
        for (int l2 = 0; l2 != number_locations; ++l2){
            tt.push_back(stoi(data[row_count][l2]));
        }
        transportation_times.push_back(tt);
    }
    ++row_count;
    // define tasks per model
    for (auto it = models.begin(); it != models.end(); ++it, ++row_count) {
        VecB tpm;
        for (int t = 0; t != number_tasks; ++t){
            tpm.push_back(stoi(data[row_count][t]) == 1 ? true : false);
        }
        (*it).Tasks_per_model = tpm;
    }
    ++row_count;
    // define task times
    for (auto it = models.begin(); it != models.end(); ++it, ++row_count) {
        int workload(0);
        VecI tt;
        for (int t = 0; t != number_tasks; ++t){
            tt.push_back(stoi(data[row_count][t]));
            workload += stoi(data[row_count][t]);
        }
        (*it).Task_times = tt;
        (*it).Workload = workload;
    }
    ++row_count;
    // define assignments
    for (auto it = locations.begin(); it != locations.end(); ++it, ++row_count) {
        VecB ass;
        for (int t = 0; t != number_tasks; ++t){
            ass.push_back(stoi(data[row_count][t]) == 1 ? true : false);
        }
        (*it).Assignments = ass;
        if ((*it).Id == start_loc_id) {
            start_loc = (*it);
        }
        if ((*it).Id == end_loc_id) {
            end_loc = (*it);
        }
    }
    ++row_count;
    // define explicit successors
    for (auto it = models.begin(); it != models.end(); ++it) {
        vector<VecB> suc;
        for (int t = 0; t != number_tasks; ++t, ++row_count){
            VecB suc_t;
            for (int t2 = 0; t2 != number_tasks; ++t2) {
                suc_t.push_back(stoi(data[row_count][t2]) == 1 ? true : false);
            }
            suc.push_back(suc_t);
        }
        (*it).Successors = suc;
    }
    ++row_count;
    // define implicit successors
    for (auto it = models.begin(); it != models.end(); ++it) {
        vector<VecB> all_suc;
        for (int t = 0; t != number_tasks; ++t, ++row_count){
            VecB all_suc_t;
            for (int t2 = 0; t2 != number_tasks; ++t2) {
                all_suc_t.push_back(stoi(data[row_count][t2]) == 1 ? true : false);
            }
            all_suc.push_back(all_suc_t);
        }
        (*it).All_successors = all_suc;
    }
    ++row_count;
    // define implicit predecessors
    for (auto it = models.begin(); it != models.end(); ++it) {
        vector<VecB> all_pre;
        for (int t = 0; t != number_tasks; ++t, ++row_count){
            VecB all_pre_t;
            for (int t2 = 0; t2 != number_tasks; ++t2) {
                all_pre_t.push_back(stoi(data[row_count][t2]) == 1 ? true : false);
            }
            all_pre.push_back(all_pre_t);
        }
        (*it).All_predecessors = all_pre;
    }
    ++row_count;
    // define preselect assignments
    for (auto it = models.begin(); it != models.end(); ++it) {
        vector<VecB> preselect_assignments;
        for (int l = 0; l != number_locations; ++l, ++row_count) {
            VecB ass;
            for (int t = 0; t != number_tasks; ++t) {
                ass.push_back(stoi(data[row_count][t]) == 1 ? true : false);
            }
            preselect_assignments.push_back(ass);
        }
        (*it).Preselect_assignments = preselect_assignments;
    }
    ++row_count;
    // define preselect all predecessors
    for (auto it = models.begin(); it != models.end(); ++it) {
        vector<VecB> preselect_all_predecessors;
        for (int t = 0; t != number_tasks; ++t, ++row_count) {
            VecB all_predecessors_mt;
            for (int t2 = 0; t2 != number_tasks; ++t2) {
                all_predecessors_mt.push_back(stoi(data[row_count][t2]) == 1 ? true : false);
            }
            preselect_all_predecessors.push_back(all_predecessors_mt);
        }
        (*it).Preselect_all_predecessors = preselect_all_predecessors;
    }
    // define preselect all successors
    for (auto it = models.begin(); it != models.end(); ++it) {
        vector<VecB> preselect_all_successors;
        for (int t = 0; t != number_tasks; ++t) {
            VecB all_successors_mt(number_tasks, false);
            for (int t2 = 0; t2 != number_tasks; ++t2) {
                if ((*it).Preselect_all_predecessors[t2][t])
                    all_successors_mt[t2] = true;
            }
            preselect_all_successors.push_back(all_successors_mt);
        }
        (*it).Preselect_all_successors = preselect_all_successors;
    }
    // define minimum layout cycle time
    for (auto m_it = models.begin(); m_it != models.end(); ++m_it) {
        // check if model exists
        bool exists(false);
        for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
            if ((*v_it).Type == (*m_it).Id) {
                exists = true;
                break;
            }
        }
        if (exists) {
            (*m_it).Min_sct = (*m_it).Workload + transportation_times[0][number_locations - 1];
        }
        else {
            (*m_it).Min_sct = 0;
        }
    }
    cout << "Instance " <<  filename << " includes " << number_locations - 2 << " locations, " << number_tasks - 2 << " tasks and " << number_vehicles << " vehicles." << endl;
}
//------------------------------------------------------------------------------------------------------------//
