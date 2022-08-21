//------------------------------------------------------------------------------------------------------------//
/* Master problem of column generation iterations. */
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
/* Create master problem as LP. */
void create_master(bool &master_activity_end, GRBModel &master, GRBVar &segment_cycle_time, vector<vector<GRBVar>> &use_route, vector<GRBVar> &dummy, vector<VecI> &active_ends,
        vector<Vehicle> &vehicles, vector<Location> &locations, int &number_locations, vector<vector<vector<VecI>>> &available_blockings, int &horizon) {
    master.set(GRB_IntParam_OutputFlag, 0);
//    master.set(GRB_IntParam_Method, 4); // makes solving deterministic??
    // define variables
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        string var_name1("dummy_" + to_string((*v_it).Id));
        dummy.push_back(master.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, GRBColumn(), var_name1));
        vector<GRBVar> use_route_v;
        for (size_t r_index = 0; r_index != (*v_it).Available_routes.size(); ++r_index) {
            string var_name("use_route_" + to_string((*v_it).Id) + "_" + to_string((*v_it).Available_routes[r_index]));
            use_route_v.push_back(master.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, GRBColumn(), var_name));
        }
        use_route.push_back(use_route_v);
    }
    segment_cycle_time = master.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, GRBColumn(), "sct");
    master.update();
    // set objective
    GRBLinExpr obj = 1.0 * segment_cycle_time;
    master.setObjective(obj, GRB_MINIMIZE);
    // define constraints
    // select one route per model and derive sct
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        GRBLinExpr cstr_select = dummy[(*v_it).Id];
        GRBLinExpr cstr_sct = 0;
        for (size_t r_index = 0; r_index != (*v_it).Available_routes.size(); ++r_index) {
            cstr_select += use_route[(*v_it).Id][r_index];
            cstr_sct += (*v_it).Route_scts[(*v_it).Available_routes[r_index]] * use_route[(*v_it).Id][r_index];
        }
        for (auto v2_it = vehicles.begin(); v2_it != vehicles.end(); ++v2_it) {
            cstr_sct += 10000000 * dummy[(*v2_it).Id];
        }
        string cstr_select_name("cstr_select_" + to_string((*v_it).Id));
        master.addConstr(cstr_select, GRB_EQUAL, 1, cstr_select_name);
        string cstr_sct_name("cstr_sct_" + to_string((*v_it).Id));
        master.addConstr(segment_cycle_time, GRB_GREATER_EQUAL, cstr_sct, cstr_sct_name);
    }
    // respect blocking
    if (master_activity_end) {
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            if ((*l_it).Id == 0 || (*l_it).Id == number_locations - 1)
                continue;
            for (int z = 0; z != horizon; ++z) {
                bool is_end(false);
                for (size_t i = 0; i != available_blockings[(*l_it).Id][z].size(); ++i) {
                    int v_id(available_blockings[(*l_it).Id][z][i][0]);
                    int r_id(available_blockings[(*l_it).Id][z][i][1]);
                    if (vehicles[v_id].Route_end_times[r_id][(*l_it).Id] - 1 == z) {
                        is_end = true;
                        break;
                    }
                }
                GRBLinExpr cstr_block = 0;
                if (is_end) {
                    active_ends.push_back({(*l_it).Id, z});
                    for (size_t i = 0; i != available_blockings[(*l_it).Id][z].size(); ++i) {
                        int v_id(available_blockings[(*l_it).Id][z][i][0]);
                        int r_id(available_blockings[(*l_it).Id][z][i][1]);
                        auto it = find(vehicles[v_id].Available_routes.begin(),  vehicles[v_id].Available_routes.end(), r_id);
                        int r_index((int) distance(vehicles[v_id].Available_routes.begin(), it));
                        cstr_block += use_route[v_id][r_index];
                    }
                }
                string cstr_block_name("cstr_block_" + to_string((*l_it).Id) + "_" + to_string(z));
                master.addConstr(cstr_block, GRB_LESS_EQUAL, 1, cstr_block_name);
            }
        }
    }
    else {
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            if ((*l_it).Id == 0 || (*l_it).Id == number_locations - 1)
                continue;
            for (int z = 0; z != horizon; ++z) {
                GRBLinExpr cstr_block = 0;
                for (size_t i = 0; i != available_blockings[(*l_it).Id][z].size(); ++i) {
                    int v_id(available_blockings[(*l_it).Id][z][i][0]);
                    int r_id(available_blockings[(*l_it).Id][z][i][1]);
                    auto it = find(vehicles[v_id].Available_routes.begin(),  vehicles[v_id].Available_routes.end(), r_id);
                    int r_index((int) distance(vehicles[v_id].Available_routes.begin(), it));
                    cstr_block += use_route[v_id][r_index];
                }
                string cstr_block_name("cstr_block_" + to_string((*l_it).Id) + "_" + to_string(z));
                master.addConstr(cstr_block, GRB_LESS_EQUAL, 1, cstr_block_name);
            }
        }
    }
    master.update();
}
//------------------------------------------------------------------------------------------------------------//
/* Derive dual cost of constraints from master problem LP solution. */
void derive_duals(GRBModel &master, vector<Vehicle> &vehicles, vector<Location> &locations, vector<VecD> &block_cost, VecD &cut_cost,
        int &number_locations, int &horizon, vector<vector<vector<VecI>>> &available_blockings, vector<vector<VecI>> &zero_half_cuts) {
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        // derive dual costs of vehicle
        (*v_it).Max_cost = master.getConstrByName("cstr_select_" + to_string((*v_it).Id)).get(GRB_DoubleAttr::GRB_DoubleAttr_Pi);
        (*v_it).Sct_cost = master.getConstrByName("cstr_sct_" + to_string((*v_it).Id)).get(GRB_DoubleAttr::GRB_DoubleAttr_Pi);
    }
    // initialize block cost
    VecD init_dummy(horizon, 0.0);
    vector<VecD> init_cost(number_locations, init_dummy);
    block_cost = init_cost;
    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
        for (int z = 0; z != horizon; ++z) {
            if ((*l_it).Id != 0 && (*l_it).Id != number_locations - 1) {
                double dual_value(-(master.getConstrByName("cstr_block_" + to_string((*l_it).Id) + "_" + to_string(z)).get(
                        GRB_DoubleAttr::GRB_DoubleAttr_Pi)));
                if (dual_value > 0) {
                    block_cost[(*l_it).Id][z] = dual_value;
                }
            }
        }
    }
    // identify duals on zero half cuts
    for (int c = 0; c != zero_half_cuts.size(); ++c) {
        cut_cost.push_back(-(master.getConstrByName("cstr_cut_" + to_string(c)).get(
                GRB_DoubleAttr::GRB_DoubleAttr_Pi)));
    }
}
//------------------------------------------------------------------------------------------------------------//