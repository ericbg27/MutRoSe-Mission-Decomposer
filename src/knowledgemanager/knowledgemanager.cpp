#include "knowledgemanager.hpp"

#include <sstream>
#include <iostream>

using namespace std;

void KnowledgeManager::set_knowledge_type(knowledge_type kt) {
    k_type = kt;
}

void KnowledgeManager::set_unique_id(string id) {
	unique_id = id;
}

void KnowledgeManager::set_type_mapping(map<string,string> tm) {
	type_mapping = tm;
}

string KnowledgeManager::get_unique_id() {
	return unique_id;
}

knowledge_type KnowledgeManager::get_knowledge_type() {
    return k_type;
}

/*
    Function: initialize_world_state
    Objective: Initialize the world state based on the world and robots knowledge. We also use the semantic mappings

    @ Input: The world state vector 
    @ Output: Void. The world state is printed in the terminal
*/ 
void print_world_state(vector<ground_literal> world_state, vector<pair<ground_literal,variant<int,float>>> world_functions) {
    cout << "World state: " << endl;
	for(ground_literal l : world_state) {
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

    cout << "World functions: " << endl;
    for(pair<ground_literal,variant<int,float>> f : world_functions) {
        string func;
        func += "(" + f.first.predicate + " ";

        unsigned int arg_num = 0;
		for(string arg : f.first.args) {
			func += arg;

			arg_num++;
			if(arg_num != f.first.args.size()) {
				func += " ";
			}
		}

        func += ") ";
		if(holds_alternative<int>(f.second)) {
        	int val = std::get<int>(f.second);
			
			func += to_string(val);
		} else {
			float val = std::get<float>(f.second);

			func += to_string(val);
		}

        cout << func << endl;
    }

	cout << endl;
}