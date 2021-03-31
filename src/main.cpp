#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <getopt.h>
#include <iostream>
#include <map>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/graph_utility.hpp>

#include "cwa.hpp"
#include "domain.hpp"
#include "hddl.hpp"
#include "hddlWriter.hpp"
#include "hpdlWriter.hpp"
#include "output.hpp"
#include "parametersplitting.hpp"
#include "parsetree.hpp"
#include "plan.hpp"
#include "shopWriter.hpp"
#include "sortexpansion.hpp"
#include "typeof.hpp"
#include "util.hpp"
#include "verify.hpp"
#include "properties.hpp"
#include "tdg.hpp"
#include "knowledgemanager.hpp"
#include "config.hpp"
#include "at.hpp"
#include "at_manager.hpp"
#include "annotmanager.hpp"
#include "missiondecomposer.hpp"
#include "instancesoutput.hpp"

using namespace std;

// declare parser function manually
void run_parser_on_file(FILE* f, char* filename);

// parsed domain data structures
bool has_typeof_predicate = false;
vector<sort_definition> sort_definitions;
vector<string> rewards_definitions;
vector<string> capabilities_definitions;
vector<predicate_definition> predicate_definitions;
vector<parsed_task> parsed_primitive;
vector<parsed_task> parsed_abstract;
map<string,vector<parsed_method> > parsed_methods;
vector<pair<predicate_definition,string>> parsed_functions;
string metric_target = dummy_function_type;

map<string,set<string>> sorts;
vector<method> methods;
vector<task> primitive_tasks;
vector<task> abstract_tasks;

map<string, task> task_name_map;

map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map;

bool mdp = false; //Variable for checking if mdp constructs are found
bool has_forall = false;
bool has_when = false;
bool has_capabilities_definitions = false;

//Structures related to robots and world model
vector<Robot> assigned_robots;

int main(int argc, char** argv) {
	cin.sync_with_stdio(false);
	cout.sync_with_stdio(false);
	int dfile = -1;
	int jsonfile = -1;
	int configfile = -1;

	for (int i = optind; i < argc; i++) {
		if (dfile == -1) dfile = i;
		else if (jsonfile == -1) jsonfile = i;
		else if (configfile == -1) configfile = i;
	}

	if (dfile == -1){
		cout << "You need to provide a domain file as input." << endl;
		return 1;
	}
	if (jsonfile == -1) {
		cout << "You need to provide a Goal Model JSON file as input." << endl;
		return 1;
	}
	if(configfile == -1) {
		cout << "You need to provide a configuration file as input." << endl;
		return 1;
	}

	FILE *domain_file = fopen(argv[dfile], "r");
	FILE *json_file = fopen(argv[jsonfile], "r");
	FILE *config_file;

	config_file = fopen(argv[configfile], "r");

	if (!domain_file) {
		cout << "I can't open " << argv[dfile] << "!" << endl;
		return 2;
	}
	if (!json_file) {
		cout << "I can't open " << argv[jsonfile] << "!" << endl;
		return 2;
	}
	if(!config_file) {
		cout << "I can't open " << argv[configfile] << "!" << endl;
		return 2;
	}

	if (has_forall) {
		cout << "MDP version does not support the FORALL operator yet." << endl;
		return 11;
	}

	namespace pt = boost::property_tree;

	pt::ptree json_root;
	pt::read_json(argv[jsonfile], json_root);

	map<string, variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> cfg;
	cfg = parse_configuration_file(argv[configfile]);

	map<string,string> type_mapping = get<map<string,string>>(cfg["type_mapping"]);
	vector<VariableMapping> variable_mapping = get<vector<VariableMapping>>(cfg["var_mapping"]);
	vector<SemanticMapping> semantic_mapping = get<vector<SemanticMapping>>(cfg["semantic_mapping"]);

	//Generate Knowledge Bases
	KnowledgeBase world_db = construct_knowledge_base("world_db", cfg);
	KnowledgeBase robots_db = construct_knowledge_base("robots_db", cfg);

	pair<string,string> output = get<pair<string,string>>(cfg["output"]);

	//Parse HDDL Domain file
	run_parser_on_file(domain_file, argv[dfile]);
		
	/*
		Add native sorts
	*/
	bool added_native_sorts = false; 
	bool has_object_sort = false;
	int cnt = 0;
	sort_definition aux;
	for(auto s : sort_definitions) {
		if(s.has_parent_sort && s.parent_sort == "object") {
			has_object_sort = true;
			aux = s;
			break;
		}
		cnt++;
	}

	if(has_object_sort) {
		aux.declared_sorts.push_back("capability");
		aux.declared_sorts.push_back("robot");
		aux.declared_sorts.push_back("robotlocation");
		aux.declared_sorts.push_back("location");
		aux.declared_sorts.push_back("robotteam");
		added_native_sorts = true;

		sort_definitions.erase(sort_definitions.begin()+cnt);
		sort_definitions.insert(sort_definitions.begin()+cnt,aux);
	}

	if(!added_native_sorts) {
		sort_definition capability_sort;
		capability_sort.declared_sorts.push_back("capability");
		capability_sort.has_parent_sort = false;

		sort_definition robot_sort;
		robot_sort.declared_sorts.push_back("robot");
		robot_sort.has_parent_sort = false;

		sort_definition robotlocation_sort;
		robotlocation_sort.declared_sorts.push_back("robotlocation");
		robotlocation_sort.has_parent_sort = false;

		sort_definition location_sort;
		location_sort.declared_sorts.push_back("location");
		location_sort.has_parent_sort = false;

		sort_definition robotteam_sort;
		robotteam_sort.declared_sorts.push_back("robotteam");
		robotteam_sort.has_parent_sort = false;

		sort_definitions.push_back(capability_sort);
		sort_definitions.push_back(robot_sort);
		sort_definitions.push_back(robotlocation_sort);
		sort_definitions.push_back(location_sort);
		sort_definitions.push_back(robotteam_sort);
	}

	//Add capabilities as constants to be used in the rest of the code
	string s = "sort_for_";
	for(string c : capabilities_definitions) {
		string remove_sort = s + c;
		sorts.erase(remove_sort);
				
		sorts["capability"].insert(c);
	}

	/*
		Add native predicates
	*/
	predicate_definition* at_pred = new predicate_definition();
	predicate_definition* hascapability_pred = new predicate_definition();
	//predicate_definition* in_pred = new predicate_definition();

	var_declaration* at_vars = new var_declaration();
	var_declaration* hascapability_vars = new var_declaration();
	//var_declaration* in_vars = new var_declaration();

	at_vars->vars.push_back(make_pair("?r","robot"));
	at_vars->vars.push_back(make_pair("?rloc","robotlocation"));
	hascapability_vars->vars.push_back(make_pair("?r","robot"));
	hascapability_vars->vars.push_back(make_pair("?c","capability"));
	//in_vars->vars.push_back(make_pair("?r", "robot"));
	//in_vars->vars.push_back(make_pair("?rt", "robotteam"));

	at_pred->name = "at";
	for(unsigned int i = 0;i < at_vars->vars.size();i++) {
		at_pred->argument_sorts.push_back(at_vars->vars[i].second);
	}
	hascapability_pred->name = "hascapability";
	for(unsigned int i = 0;i < hascapability_vars->vars.size();i++) {
		hascapability_pred->argument_sorts.push_back(hascapability_vars->vars[i].second);
	}
	/*in_pred->name = "in";
	for(unsigned int i = 0;i < in_vars->vars.size();i++) {
		in_pred->argument_sorts.push_back(in_vars->vars[i].second);
	}*/

	expand_sorts();
	/*
		Additional variables are added in lines 96-114 (14/10) of domain.cpp (in function flatten_mdp_tasks)
		These are added for every constant, that's why capabilities appear with ?var_for_{capability name} and sort_for_{capability name}
		- Added in var_or_const_list definition in hddl.y
	*/

	/*
		For HDDL parsing we need to go through the abstract tasks and generate the TDG for each one of them. With the TDGs
		in hand, we can find out which tasks form the abstract task
	*/

	flatten_mdp_tasks();

	parsed_method_to_data_structures(false, false, false);

	/*
		Goal Model parsing and generation of Abstract tasks instances
	*/

	GMGraph gm;

	gm = graph_from_property_tree(json_root);

	/*
		TODO: Add flag which will indicate if we need to verify or not our constructs (deal with errors)
	*/
	check_gm_validity(gm);

	print_gm_nodes_info(gm);

	check_undefined_number_of_robots(gm, abstract_tasks, sort_definitions);

	//For now, only one high-level location type allowed
	string location_type = get<vector<string>>(cfg["location_types"]).at(0);

	map<string,vector<AbstractTask>> at_instances;
	
	at_instances = generate_at_instances(abstract_tasks, gm, location_type, world_db, gm_var_map, variable_mapping);

	print_at_instances_info(at_instances);

	print_gm_var_map_info(gm_var_map);

	/*
		Tasks generated by methods preconditions (which are preceded by __method_precondition_) contain in the vars attribute
		only variables used in the preconditions of the method. They come first in the method's task list.

		Example:
		Method

		(:method robot-sanitization
        	:parameters (?r - CleanerRobot ?srm - room ?rloc - location ?c - capability)
        	:task (RobotSanitization ?r ?srm ?rloc ?c)
        	:precondition (and
            	(hascapability ?r ?c)
       		)
        	:subtasks (and
            	(sanitize-robot ?r ?srm ?rloc)
        	)
		)

		generates primitive task

		(:action
			:parameters (?r - CleanerRobot ?c - capability)
			:precondition (and
				(hascapability ?r ?c)
			)
			:effect ()
		)
	*/

	map<string,vector<vector<task>>> at_decomposition_paths;

	for(task at : abstract_tasks) {
		TDG t(at, abstract_tasks, primitive_tasks, methods);
			
		at_decomposition_paths[at.name] = t.retrieve_possible_decompositions();
	}

	print_at_paths_info(at_decomposition_paths);

	initialize_objects(world_db, robots_db, sorts, location_type, at_instances, type_mapping);	

	initialize_world_state(robots_db, world_db, init, init_functions, semantic_mapping, type_mapping, sorts);

	print_world_state(init);

	general_annot* gmannot = retrieve_gm_annot(gm, world_db.get_knowledge(), location_type, at_instances);

	rename_at_instances_in_runtime_annot(gmannot, at_instances);

	print_runtime_annot_from_general_annot(gmannot);		

	/*
		We need to associate to trim decomposition paths only to those paths that are allowed

		-> For this we need to verify AT ordering in the GM
			- With this we know how to check the initial state of the world for each and every instance of each AT
			- We can use the goal model runtime annotation for this
		-> With trimmed decomposition paths we know how many extra instances of the AT's we need to create
		-> This is the information we need to finally know all of the AT instances we can execute
			- When task allocation is performed, only one way of decomposing each AT will be chosen
			
		-> Let us note that we have two steps in AT generation
			- One is analyzing the Goal Model and its forAll statements (in the future iterate statements possibly), generating instances that
			NEED to be executed
			- The second step is generation based on possible paths of decomposition, where we have a decision to be made between one of the instances
			generated from this process
	*/

	ATGraph mission_decomposition = build_at_graph(at_instances, at_decomposition_paths, gmannot, gm, init, gm_var_map, world_db, semantic_mapping);

	print_mission_decomposition(mission_decomposition);

	generate_instances_output(mission_decomposition,gm,output,init,semantic_mapping,sorts,sort_definitions,predicate_definitions);
}