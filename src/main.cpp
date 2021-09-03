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

#include "utils/cwa.hpp"
#include "utils/domain.hpp"
#include "utils/output.hpp"
#include "utils/parametersplitting.hpp"
#include "utils/parsetree.hpp"
#include "utils/plan.hpp"
#include "utils/sortexpansion.hpp"
#include "utils/typeof.hpp"
#include "utils/util.hpp"
#include "utils/verify.hpp"
#include "utils/properties.hpp"
#include "hddl/hddl.hpp"
#include "tdg/tdg.hpp"
#include "knowledgemanager/knowledgemanager.hpp"
#include "knowledgemanager/fileknowledgemanager.hpp"
#include "knowledgemanager/knowledgemanagerfactory.hpp"
#include "config/config.hpp"
#include "atmanager/at_manager.hpp"
#include "annotmanager/annotmanager.hpp"
#include "missiondecomposer/missiondecomposer.hpp"
#include "outputgenerator/outputgenerator.hpp"
#include "outputgenerator/xmloutputgenerator.hpp"
#include "outputgenerator/fileoutputgeneratorfactory.hpp"
#include "configchecker/configchecker.hpp"

using namespace std;

const string verbose_command = "-v";
const string pretty_print_command = "-p";

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
set<string> robot_related_sorts;
vector<method> methods;
vector<task> primitive_tasks;
vector<task> abstract_tasks;

map<string, task> task_name_map;

map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map;

const string knowledge_unique_id = "name";

bool mdp = false; //Variable for checking if mdp constructs are found
bool has_forall = false;
bool has_when = false;
bool has_capabilities_definitions = false;
bool verbose = false;
bool pretty_print = false;

int main(int argc, char** argv) {
	cin.sync_with_stdio(false);
	cout.sync_with_stdio(false);
	int dfile = -1;
	int jsonfile = -1;
	int configfile = -1;
	int knowledgefile = -1;
	vector<int> options;

	for (int i = optind; i < argc; i++) {
		if (dfile == -1) dfile = i;
		else if (jsonfile == -1) jsonfile = i;
		else if (configfile == -1) configfile = i;
		else if (knowledgefile == -1) knowledgefile = i;
	}

	for(int i = knowledgefile+1; i < argc; i++) {
		options.push_back(i);
	}

	bool option_not_found = false;
	for(int option : options) {
		string opt(argv[option]);

		if(opt == verbose_command) {
			verbose = true;
		} else if(opt == pretty_print_command) {
			pretty_print = true;
		} else {
			if(!option_not_found) {
				std::cout << std::endl;
			}
			std::cout << "Unknown option [" + opt + "]" << std::endl;
			option_not_found = true;
		}
	}

	if(verbose && pretty_print) {
		std::cout << "Options -v and -p cannot be used together!" << std::endl;

		return 1;
	}

	if(option_not_found) {
		std::cout << std::endl;
	}

	if(dfile == -1){
		cout << "You need to provide a domain file as input." << endl;
		return 1;
	}
	if(jsonfile == -1) {
		cout << "You need to provide a Goal Model JSON file as input." << endl;
		return 1;
	}
	if(configfile == -1) {
		cout << "You need to provide a configuration file as input." << endl;
		return 1;
	}
	if(knowledgefile == -1) {
		cout << "You need to provide a knowledge file as input" << endl;
		return 1;
	}

	FILE *domain_file = fopen(argv[dfile], "r");
	FILE *json_file = fopen(argv[jsonfile], "r");
	FILE *config_file = fopen(argv[configfile], "r");
	FILE *knowledge_file = fopen(argv[knowledgefile], "r");

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
	if(!knowledgefile) {
		cout << "I can't open " << argv[knowledgefile] << "!" << endl;
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

	ConfigManager cfg_manager;
	cfg = cfg_manager.parse_configuration_file(argv[configfile]);

	map<string,string> type_mapping = std::get<map<string,string>>(cfg["type_mapping"]);

	vector<VariableMapping> variable_mapping = std::get<vector<VariableMapping>>(cfg["var_mapping"]);
	vector<SemanticMapping> semantic_mapping = std::get<vector<SemanticMapping>>(cfg["semantic_mapping"]);
	
	vector<string> high_level_loc_types = std::get<vector<string>>(cfg["location_types"]);

	//Generate Knowledge Bases and Knowledge Manager
	KnowledgeManagerFactory k_manager_factory;
	shared_ptr<KnowledgeManager> knowledge_manager = k_manager_factory.create_knowledge_manager(cfg, knowledge_unique_id, type_mapping);
	knowledge_manager->construct_knowledge_base(argv[knowledgefile]);

	vector<string> output = std::get<vector<string>>(cfg["output"]);

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

	/*
		Add capabilities as constants to be used in the rest of the code
	*/
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

	var_declaration* at_vars = new var_declaration();
	var_declaration* hascapability_vars = new var_declaration();

	at_vars->vars.push_back(make_pair("?r","robot"));
	at_vars->vars.push_back(make_pair("?rloc","robotlocation"));
	hascapability_vars->vars.push_back(make_pair("?r","robot"));
	hascapability_vars->vars.push_back(make_pair("?c","capability"));

	at_pred->name = "at";
	for(unsigned int i = 0;i < at_vars->vars.size();i++) {
		at_pred->argument_sorts.push_back(at_vars->vars[i].second);
	}
	hascapability_pred->name = "hascapability";
	for(unsigned int i = 0;i < hascapability_vars->vars.size();i++) {
		hascapability_pred->argument_sorts.push_back(hascapability_vars->vars[i].second);
	}

	if(sort_definitions.size() > 0) {
		vector<sort_definition>::iterator sort_def_it = sort_definitions.begin();
		sort_definition current_sort = *sort_def_it;

		while(sort_def_it != sort_definitions.end()) {
			bool next = false;

			if(current_sort.has_parent_sort && (current_sort.parent_sort == hddl_robot_type || current_sort.parent_sort == hddl_robotteam_type)) {
				for(string sort : current_sort.declared_sorts) {
					robot_related_sorts.insert(sort);
				}

				next = true;
			} else if(current_sort.has_parent_sort && current_sort.parent_sort != hddl_robot_type && current_sort.parent_sort != hddl_robotteam_type) {
				bool found_def = false;
				
				sort_definition aux;
				for(sort_definition s : sort_definitions) {
					if(std::find(s.declared_sorts.begin(), s.declared_sorts.end(), current_sort.parent_sort) != s.declared_sorts.end()) {
						aux = s;
						found_def = true;

						break;
					}
				}

				if(found_def) {
					current_sort = aux;
				} else {
					next = true;
				}
			} else {
				next = true;
			}

			if(next) {
				sort_def_it++;

				if(sort_def_it != sort_definitions.end()) {
					current_sort = *sort_def_it;
				}
			}
		}
	}

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

	check_config(variable_mapping, type_mapping, gm, abstract_tasks, semantic_mapping, high_level_loc_types, predicate_definitions);

	check_gm_validity(gm);

	if(verbose) {
		print_gm(gm);

		print_gm_nodes_info(gm);
	}

	check_undefined_number_of_robots(gm, abstract_tasks, sort_definitions);

	ATManagerFactory at_manager_factory;
	shared_ptr<ATManager> at_manager_ptr = at_manager_factory.create_at_manager(knowledge_manager, abstract_tasks, gm, high_level_loc_types);

	map<string,vector<AbstractTask>> at_instances;

	if(at_manager_ptr->get_at_manager_type() == ATFILE) {
		FileKnowledgeATManager* at_manager = dynamic_cast<FileKnowledgeATManager*>(at_manager_ptr.get());

		FileKnowledgeManager* aux = dynamic_cast<FileKnowledgeManager*>(knowledge_manager.get());
		at_manager->set_fk_manager(aux);

		at_instances = at_manager->generate_at_instances(gm_var_map,variable_mapping);
	}

	if(verbose) {
		print_at_instances_info(at_instances);

		print_gm_var_map_info(gm_var_map);
	}

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

	map<string,vector<DecompositionPath>> at_decomposition_paths;

	for(task at : abstract_tasks) {
		TDG t(at, abstract_tasks, primitive_tasks, methods, verbose);

		at_decomposition_paths[at.name] = t.retrieve_possible_decompositions();
	}

	if(verbose) {
		print_at_paths_info(at_decomposition_paths);
	}

	knowledge_manager->initialize_objects(sorts, high_level_loc_types, at_instances);
	knowledge_manager->initialize_world_state(init, init_functions, semantic_mapping, sorts);

	if(verbose) {
		print_world_state(init,init_functions);
	}

	AnnotManagerFactory annot_manager_factory;
	shared_ptr<AnnotManager> annot_manager_ptr = annot_manager_factory.create_annot_manager(knowledge_manager, gm, high_level_loc_types, at_instances);

	general_annot* gmannot;

	if(annot_manager_ptr->get_annot_manager_type() == FILEANNOTMANAGER) {
		FileKnowledgeAnnotManager* annot_manager = dynamic_cast<FileKnowledgeAnnotManager*>(annot_manager_ptr.get());
		
		FileKnowledgeManager* aux = dynamic_cast<FileKnowledgeManager*>(knowledge_manager.get());
		annot_manager->set_fk_manager(aux);

		gmannot = annot_manager->retrieve_gm_annot();
	}

	rename_at_instances_in_runtime_annot(gmannot, at_instances, gm);

	if(verbose) {
		print_runtime_annot_from_general_annot(gmannot);
	}

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

	MissionDecomposerFactory mission_decomposer_factory;
	shared_ptr<MissionDecomposer> mission_decomposer_ptr = mission_decomposer_factory.create_mission_decomposer(knowledge_manager, init, init_functions, at_decomposition_paths, at_instances, gmannot, gm, verbose);
	
	ATGraph mission_decomposition;

	if(mission_decomposer_ptr->get_mission_decomposer_type() == FILEMISSIONDECOMPOSER) {
		FileKnowledgeMissionDecomposer* mission_decomposer = dynamic_cast<FileKnowledgeMissionDecomposer*>(mission_decomposer_ptr.get());

		FileKnowledgeManager* aux = dynamic_cast<FileKnowledgeManager*>(knowledge_manager.get());
		mission_decomposer->set_fk_manager(aux);

		mission_decomposition = mission_decomposer->build_at_graph(gm_var_map, semantic_mapping);
	}

	if(verbose) {
		print_mission_decomposition(mission_decomposition); 
	}

	if(output.at(0) == "FILE") {
		FileOutputGeneratorFactory output_gen_factory;

		pair<string,string> file_output_data = std::make_pair(output.at(1),output.at(2));
		std::shared_ptr<FileOutputGenerator> output_generator_ptr = output_gen_factory.create_file_output_generator(gm, mission_decomposition, init, init_functions, file_output_data, verbose, pretty_print);

		if(output_generator_ptr->get_file_output_generator_type() == XMLFILEOUTGEN) {
			XMLOutputGenerator* output_generator = dynamic_cast<XMLOutputGenerator*>(output_generator_ptr.get());

			output_generator->generate_instances_output(semantic_mapping, sorts, sort_definitions, predicate_definitions, gm_var_map, robot_related_sorts);
		} else if(output_generator_ptr->get_file_output_generator_type() == JSONFILEOUTGEN) {
			JSONOutputGenerator* output_generator = dynamic_cast<JSONOutputGenerator*>(output_generator_ptr.get());

			output_generator->generate_instances_output(semantic_mapping, sorts, sort_definitions, predicate_definitions, gm_var_map, robot_related_sorts);
		}
	}
}