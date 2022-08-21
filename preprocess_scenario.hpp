//------------------------------------------------------------------------------------------------------------//
/* Determine available task-location combinations for all models in scenarios. */
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
/* Preprocess scenario NF.*/
void preprocess_scenario_nf(vector<Model> &models, int &number_locations) {
    for (auto m_it = models.begin(); m_it != models.end(); ++m_it) {
        VecI empty_vector({});
        vector<VecI> scenario_tlc(number_locations, empty_vector);
        for (auto r_it = (*m_it).Preselect_route.begin(); r_it != (*m_it).Preselect_route.end(); ++r_it) {
            scenario_tlc[(*r_it)[0]].push_back((*r_it)[1]);
        }
        (*m_it).Scenario_task_location_combinations = scenario_tlc;
        (*m_it).Scenario_all_successors = (*m_it).Preselect_all_successors;
        (*m_it).Scenario_all_predecessors = (*m_it).Preselect_all_predecessors;
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Preprocess scenario OF.*/
void preprocess_scenario_of(vector<Model> &models, int &number_locations) {
    for (auto m_it = models.begin(); m_it != models.end(); ++m_it) {
        VecI empty_vector({});
        vector<VecI> scenario_tlc(number_locations, empty_vector);
        for (auto r_it = (*m_it).Preselect_route.begin(); r_it != (*m_it).Preselect_route.end(); ++r_it) {
            scenario_tlc[(*r_it)[0]].push_back((*r_it)[1]);
        }
        (*m_it).Scenario_task_location_combinations = scenario_tlc;
        (*m_it).Scenario_all_successors = (*m_it).All_successors;
        (*m_it).Scenario_all_predecessors = (*m_it).All_predecessors;
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Preprocess scenario RF.*/
void preprocess_scenario_rf(vector<Model> &models, vector<Location> &locations, int &number_locations) {
    for (auto m_it = models.begin(); m_it != models.end(); ++m_it) {
        VecI empty_vector({});
        vector<VecI> scenario_tlc(number_locations, empty_vector);
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            for (size_t r = 0; r != (*m_it).Possible_task_location_combinations[(*l_it).Id].size(); ++r) {
                scenario_tlc[(*l_it).Id].push_back((int) r);
            }
        }
        (*m_it).Scenario_task_location_combinations = scenario_tlc;
        (*m_it).Scenario_all_successors = (*m_it).Preselect_all_successors;
        (*m_it).Scenario_all_predecessors = (*m_it).Preselect_all_predecessors;
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Preprocess scenario FF.*/
void preprocess_scenario_ff(vector<Model> &models, vector<Location> &locations, int &number_locations) {
    for (auto m_it = models.begin(); m_it != models.end(); ++m_it) {
        VecI empty_vector({});
        vector<VecI> scenario_tlc(number_locations, empty_vector);
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            for (size_t r = 0; r != (*m_it).Possible_task_location_combinations[(*l_it).Id].size(); ++r) {
                scenario_tlc[(*l_it).Id].push_back((int) r);
            }
        }
        (*m_it).Scenario_task_location_combinations = scenario_tlc;
        (*m_it).Scenario_all_successors = (*m_it).All_successors;
        (*m_it).Scenario_all_predecessors = (*m_it).All_predecessors;
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Create copy of original vehicles at start of new flexibility scenario. */
void initialize_models_for_scenario(vector<Model> &models, vector<Location> &locations, int &number_locations,
        int &scenario, string &string_scenario) {
    if (scenario == 0) {
        string_scenario = "NF";
        preprocess_scenario_nf(models, number_locations);
    }
    if (scenario == 1) {
        string_scenario = "OF";
        preprocess_scenario_of(models, number_locations);
    }
    else if (scenario == 2) {
        string_scenario = "RF";
        preprocess_scenario_rf(models, locations, number_locations);
    }
    else if (scenario == 3) {
        string_scenario = "FF";
        preprocess_scenario_ff(models, locations, number_locations);
    }
}
//------------------------------------------------------------------------------------------------------------//
