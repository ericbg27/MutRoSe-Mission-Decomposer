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
	int pfile = -1;
	int doutfile = -1;
	int poutfile = -1;
	int jsonfile = 0;
	int configfile = 0;
	bool splitParameters = true;
	bool compileConditionalEffects = true;
	bool linearConditionalEffectExpansion = false;
	bool encodeDisjunctivePreconditionsInMethods = false;
	bool compileGoalIntoAction = false;
	
	bool shopOutput = false;
	bool hpdlOutput = false;
	bool hddlOutput = false;
	bool internalHDDLOutput = false;
	bool mdpOutput = false;
	bool prismOutput = false;
	bool lenientVerify = false;
	bool verboseOutput = false;
	bool verifyPlan = false;
	bool useOrderInPlanVerification = true;
	bool convertPlan = false;
	bool showProperties = false;
	int verbosity = 0;
	
	struct option options[] = {
		{"no-split-parameters"                    , no_argument,       NULL,   's'},
		{"keep-conditional-effects"               , no_argument,       NULL,   'k'},
		{"linear-conditional-effect"              , no_argument,       NULL,   'L'},
		{"encode-disjunctive_preconditions-in-htn", no_argument,       NULL,   'D'},
		{"compile-goal"							  , no_argument,       NULL,   'g'},
		
		{"shop"                                   , no_argument,       NULL,   'S'},
		{"shop2"                                  , no_argument,       NULL,   'S'},
		{"shop1"                                  , no_argument,       NULL,   '1'},
		{"hpdl"                                   , no_argument,       NULL,   'H'},
		{"hddl"                                   , no_argument,       NULL,   'h'},
		{"hddl-internal"                          , no_argument,       NULL,   'i'},
		{"mdp"					  				  , no_argument,	   NULL,   'm'},
		{"prism"								  , no_argument,	   NULL,   'M'},
		
		{"panda-converter"                        , no_argument,       NULL,   'c'},
		{"verify"                                 , optional_argument, NULL,   'v'},
		{"vverify"                                , no_argument,       NULL,   'V'},
		{"vvverify"                               , no_argument,       NULL,   'W'},
		{"lenient"                                , no_argument,       NULL,   'l'},
		{"verify-no-order"                        , no_argument,       NULL,   'o'},
		
		{"no-color"                               , no_argument,       NULL,   'C'},
		{"debug"                                  , optional_argument, NULL,   'd'},
		
		{"properties"                             , optional_argument, NULL,   'p'},
		
		{NULL                                     , 0,                 NULL,   0},
	};

	bool optionsValid = true;
	while (true) {
		int c = getopt_long_only (argc, argv, "sS1HcvVWoCdkhilpLDgM", options, NULL);
		if (c == -1)
			break;
		if (c == '?' || c == ':'){
			// Invalid option; getopt_long () will print an error message
			optionsValid = false;
			continue;
		}

		if (c == 's') splitParameters = false;
		else if (c == 'k') compileConditionalEffects = false;
		else if (c == 'L') { compileConditionalEffects = false; linearConditionalEffectExpansion = true; }
		else if (c == 'D') encodeDisjunctivePreconditionsInMethods = true;
		else if (c == 'g') compileGoalIntoAction = true;
		else if (c == 'S') shopOutput = true;
		else if (c == '1') { shopOutput = true; shop_1_compatability_mode = true; }
	   	else if (c == 'H') hpdlOutput = true;
	   	else if (c == 'h') hddlOutput = true;
	   	else if (c == 'i') { hddlOutput = true; internalHDDLOutput = true; }
		else if (c == 'M') {mdpOutput = true; prismOutput = true; compileConditionalEffects = false; pfile = 0; doutfile = 0; poutfile = 0; jsonfile = -1; configfile = -1;}
		else if (c == 'c') convertPlan = true;
		else if (c == 'v') {
			verifyPlan = true;
			if (optarg) verbosity = atoi(optarg);
		} else if (c == 'V') { verifyPlan = true; verbosity = 1; }
		else if (c == 'W') { verifyPlan = true; verbosity = 2; }
		else if (c == 'o') { verifyPlan = true; useOrderInPlanVerification = false; }
		else if (c == 'l') { verifyPlan = true; lenientVerify = true; }
		else if (c == 'C') no_colors_in_output = true;
		else if (c == 'p') showProperties = true;
		else if (c == 'd') {
		   	verboseOutput = true;
			if (optarg) verbosity = atoi(optarg);
		}
	}
	if (!optionsValid) {
		cout << "Invalid options. Exiting." << endl;
		return 1;
	}

	for (int i = optind; i < argc; i++) {
		if (dfile == -1) dfile = i;
		else if (pfile == -1) pfile = i;
		else if (doutfile == -1) doutfile = i;
		else if (poutfile == -1) poutfile = i;
		else if (jsonfile == -1) jsonfile = i;
		else if (configfile == -1) configfile = i;
	}

	if(!mdpOutput) {
		if (dfile == -1){
			if (convertPlan)
				cout << "You need to provide a plan as input." << endl;
			else
				cout << "You need to provide a domain and problem file as input." << endl;
			return 1;
		}

		// if we want to simplify a plan, just parse nothing
		if (convertPlan){
			ifstream * plan   = new ifstream(argv[dfile]);
			ostream * outplan = &cout;
			if (pfile != -1){
				ofstream * of  = new ofstream(argv[pfile]);
				if (!of->is_open()){
					cout << "I can't open " << argv[pfile] << "!" << endl;
					return 2;
				}
				outplan = of;
			}
			
			
			convert_plan(*plan, *outplan);
			return 0;
		}

		if (pfile == -1 && !convertPlan){
			cout << "You need to provide a domain and problem file as input." << endl;
			return 1;
		}

		// open c-style file handle
		FILE *domain_file = fopen(argv[dfile], "r");
		FILE *problem_file = fopen(argv[pfile], "r");

		if (!domain_file) {
			cout << "I can't open " << argv[dfile] << "!" << endl;
			return 2;
		}
		if (!problem_file) {
			cout << "I can't open " << argv[pfile] << "!" << endl;
			return 2;
		}
		if (!shopOutput && !hpdlOutput && !hddlOutput && poutfile != -1){
			cout << "For ordinary pandaPI output, you may only specify one output file, but you specified two: " << argv[doutfile] << " and " << argv[poutfile] << endl;
		}
		// parsing of command line arguments has been completed	
		
		// parse the domain file
		run_parser_on_file(domain_file, argv[dfile]);
		run_parser_on_file(problem_file, argv[pfile]);

		if (showProperties){
			printProperties();
			return 0;
		}

		if(mdp) {
			cout << "Probabilistic initializations, rewards and reliabilities are not supported in the chosen option" << endl;
			return 10;
		}

		if (!hpdlOutput) expand_sorts(); // add constants to all sorts
		
		// handle typeof-predicate
		if (!hpdlOutput && has_typeof_predicate) create_typeof();

		if (compileGoalIntoAction) compile_goal_into_action();


		// do not preprocess the instance at all if we are validating a solution
		if (verifyPlan){
			ifstream * plan  = new ifstream(argv[doutfile]);
			bool result = verify_plan(*plan, useOrderInPlanVerification, lenientVerify, verbosity);
			cout << "Plan verification result: ";
			if (result) cout << color(COLOR_GREEN,"true",MODE_BOLD);
			else cout << color(COLOR_RED,"false",MODE_BOLD);
			cout << endl;
			return 0;
		}

		if (!hpdlOutput) {
			// flatten all primitive tasks
			flatten_tasks(compileConditionalEffects, linearConditionalEffectExpansion, encodeDisjunctivePreconditionsInMethods);
			// .. and the goal
			flatten_goal();
			// create appropriate methods and expand method preconditions
			parsed_method_to_data_structures(compileConditionalEffects, linearConditionalEffectExpansion, encodeDisjunctivePreconditionsInMethods);
		}

		if (shopOutput || hpdlOutput){
			// produce streams for output
			ostream * dout = &cout;
			ostream * pout = &cout;
			if (doutfile != -1){
				ofstream * df  = new ofstream(argv[doutfile]);
				if (!df->is_open()){
					cout << "I can't open " << argv[doutfile] << "!" << endl;
					return 2;
				}
				dout = df;
			}
			if (poutfile != -1){
				ofstream * pf  = new ofstream(argv[poutfile]);
				if (!pf->is_open()){
					cout << "I can't open " << argv[poutfile] << "!" << endl;
					return 2;
				}
				pout = pf;
			}
			if (shopOutput)	write_instance_as_SHOP(*dout,*pout);
			if (hpdlOutput)	write_instance_as_HPDL(*dout,*pout);
			return 0;
		}


		// split methods with independent parameters to reduce size of grounding
		if (splitParameters) split_independent_parameters();
		// cwa, but only if we actually want to compile negative preconditions
		if (!hpdlOutput || internalHDDLOutput) compute_cwa();
		// simplify constraints as far as possible
		reduce_constraints();
		clean_up_sorts();
		remove_unnecessary_predicates();

		// write to output
		if (verboseOutput) verbose_output(verbosity);
		else if (hddlOutput) {
			// produce streams for output
			ostream * dout = &cout;
			ostream * pout = &cout;
			if (doutfile != -1){
				ofstream * df  = new ofstream(argv[doutfile]);
				if (!df->is_open()){
					cout << "I can't open " << argv[doutfile] << "!" << endl;
					return 2;
				}
				dout = df;
			}
			if (poutfile != -1){
				ofstream * pf  = new ofstream(argv[poutfile]);
				if (!pf->is_open()){
					cout << "I can't open " << argv[poutfile] << "!" << endl;
					return 2;
				}
				pout = pf;
			}
			hddl_output(*dout,*pout, internalHDDLOutput);
		} else {
			ostream * dout = &cout;
			if (doutfile != -1){
				ofstream * df  = new ofstream(argv[doutfile]);
				if (!df->is_open()){
					cout << "I can't open " << argv[doutfile] << "!" << endl;
					return 2;
				}
				dout = df;
			}
			simple_hddl_output(*dout);
		}
	} else {
		if (dfile == -1){
			cout << "You need to provide a domain file as input." << endl;
			return 1;
		}
		if (pfile != 0) {
			cout << "Problem files are not supported in this option" << endl;
			return 1;
		}
		if (jsonfile == -1) {
			cout << "You need to provide a Goal Model JSON file as input." << endl;
			return 1;
		}
		if(configfile == -1 && prismOutput) {
			cout << "You need to provide an assignments file as input." << endl;
		}

		FILE *domain_file = fopen(argv[dfile], "r");
		FILE *json_file = fopen(argv[jsonfile], "r");
		FILE *config_file;
		if(prismOutput) {
			config_file = fopen(argv[configfile], "r");
		}

		if (!domain_file) {
			cout << "I can't open " << argv[dfile] << "!" << endl;
			return 2;
		}
		if (!json_file) {
			cout << "I can't open " << argv[jsonfile] << "!" << endl;
			return 2;
		}
		if(!config_file && prismOutput) {
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

		parsed_method_to_data_structures(compileConditionalEffects, linearConditionalEffectExpansion, encodeDisjunctivePreconditionsInMethods);

		/*
			Goal Model parsing and generation of Abstract tasks instances
		*/

		GMGraph gm;

		gm = graph_from_property_tree(json_root);

		GMGraph::vertex_iterator i1, end1;
		GMGraph::adjacency_iterator ai1, a_end1;

		for(boost::tie(i1,end1) = vertices(gm); i1 != end1; ++i1) {
			VertexData node = gm[*i1];

			std::cout << "Node: " << node.text << std::endl;
			std::cout << "Context: " << std::endl;

			Context c;
			
			if(node.custom_props.find("CreationCondition") != node.custom_props.end()) {
				c = get<Context>(node.custom_props["CreationCondition"]);

				std::cout << "\tType: " << c.type << std::endl;
				std::cout << "\tCondition: " << c.condition << std::endl;
			} else {
				std::cout << "\tNo Context" << std::endl;
			}
			
			std::cout << std::endl;
		}

		check_undefined_number_of_robots(gm, abstract_tasks, sort_definitions);

		//For now, only one high-level location type allowed
		string location_type = get<vector<string>>(cfg["location_types"]).at(0);

		map<string,vector<AbstractTask>> at_instances;
		
		at_instances = generate_at_instances(abstract_tasks, gm, world_db.get_knowledge(), location_type, world_db, gm_var_map, variable_mapping);

		map<string,vector<AbstractTask>>::iterator at_it;
		cout << "AT instances:" << endl;
		for(at_it = at_instances.begin();at_it != at_instances.end();++at_it) {
			cout << "AT name: " << at_it->first << endl;
			for(AbstractTask inst : at_it->second) {
				cout << "ID: " << inst.id << endl;
				cout << "Name: " << inst.name << endl;
				cout << "Variable Mappings:" << endl;
				for(auto var_map : inst.variable_mapping) {
					cout << var_map.second << ": " << var_map.first << endl;
				}
				cout << "Triggering Events:" << endl;
				for(string event : inst.triggering_events) {
					cout << event << ", ";
				}
				cout << endl;
			}
		}

		map<string, variant<pair<string,string>,pair<vector<string>,string>>>::iterator gm_var_it;
		for(gm_var_it = gm_var_map.begin();gm_var_it != gm_var_map.end();++gm_var_it) {
			cout << "Var name: " << gm_var_it->first << endl;
			if(holds_alternative<pair<vector<string>,string>>(gm_var_it->second)) {
				cout << "Mapping: [";
				unsigned int cnt = 0;
				vector<string> val_vec = get<pair<vector<string>,string>>(gm_var_it->second).first;
				for(string val : val_vec) {
					if(cnt == val_vec.size()-1) {
						cout << val << "]" << endl << endl;
					} else {
						cout << val << ",";
					}
					cnt++;
				} 
			} else {
				cout << "Mapping: " << get<pair<string,string>>(gm_var_it->second).first << endl << endl;
			}
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

		map<string,vector<vector<task>>> at_decomposition_paths;

		for(task at : abstract_tasks) {
			TDG t(at, abstract_tasks, primitive_tasks, methods);
			
			at_decomposition_paths[at.name] = t.retrieve_possible_decompositions();
		}

		map<string,vector<vector<task>>>::iterator at_paths_it;

		for(at_paths_it = at_decomposition_paths.begin();at_paths_it != at_decomposition_paths.end();++at_paths_it) {
			cout << "Abstract task " << at_paths_it->first << " decomposition paths:" << endl;

			for(auto path : at_paths_it->second) {
				cout << "#####################################" << endl;
				cout << "Path: ";
				for(auto t : path) {
					cout << t.name;
					if(t.name != path.back().name) {
						cout << " -> ";
					} else {
						cout << endl;
					}
				}
				cout << "#####################################" << endl;
			}

			cout << endl;
		}

		initialize_objects(world_db,robots_db,sorts, location_type, at_instances, type_mapping);

		map<string,set<string>>::iterator sorts_it;
		for(sorts_it = sorts.begin();sorts_it != sorts.end();++sorts_it) {
			cout << "Sort Name: " << sorts_it->first << endl;
			cout << "Children: " << endl;
			set<string>::iterator ch_it;
			for(ch_it = sorts_it->second.begin();ch_it != sorts_it->second.end();++ch_it) {
				cout << *(ch_it) << endl;
			}	
		}

		cout << endl;		

		initialize_world_state(robots_db, world_db, init, init_functions, semantic_mapping, type_mapping, sorts);

		cout << "Initial world state: " << endl;
		for(ground_literal l : init) {
			string state;
			if(!l.positive) {
				state += "not ";
			}
			state += l.predicate + " ";

			unsigned int arg_num = 0;
			for(string arg : l.args) {
				state += arg;

				arg_num++;
				if(arg_num != l.args.size()) {
					state += " ";
				}
			}

			cout << state << endl;
		}

		cout << endl;

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

		ATGraph::vertex_iterator i, end;
		ATGraph::adjacency_iterator ai, a_end;

		for(boost::tie(i,end) = vertices(mission_decomposition); i != end; ++i) {
			ATNode node = mission_decomposition[*i];
			if(holds_alternative<AbstractTask>(node.content)) {
				std::cout << get<AbstractTask>(node.content).id << " --> ";
			} else if(holds_alternative<string>(node.content)) {
				std::cout << get<string>(node.content) << " --> ";
			} else {
				std::cout << get<Decomposition>(node.content).id << " --> ";	
			}

			for(boost::tie(ai,a_end) = adjacent_vertices(*i,mission_decomposition); ai != a_end;++ai) {
				ATNode a_node = mission_decomposition[*ai];
				if(holds_alternative<AbstractTask>(a_node.content)) {
					std::cout << get<AbstractTask>(a_node.content).id << " ";
				} else if(holds_alternative<string>(a_node.content)) {
					std::cout << get<string>(a_node.content) << " ";
				} else {
					std::cout << get<Decomposition>(a_node.content).id << " ";
				}
			}	
			std::cout << std::endl;
		}

		cout << endl;
		generate_instances_output(mission_decomposition,gm,output,init,semantic_mapping,sorts,sort_definitions,predicate_definitions);
	}
}
