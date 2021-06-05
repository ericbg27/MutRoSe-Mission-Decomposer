#include "knowledgemanager.hpp"

#include <sstream>
#include <iostream>

using namespace std;

void KnowledgeManager::set_knowledge_type(knowledge_type kt) {
    k_type = kt;
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
void print_world_state(vector<ground_literal> world_state, vector<pair<ground_literal,int>> world_functions) {
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
    for(pair<ground_literal,int> f : world_functions) {
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
        func += to_string(f.second);

        cout << func << endl;
    }

	cout << endl;
}