#include "util.hpp"

#include <iostream>

bool no_colors_in_output = false;

string color(Color color, string text, Mode m, Color background){
	if (no_colors_in_output) return text;
	return string ()
		+ "\033[" +std::to_string (m)+ ";" + std::to_string (30 + color) + "m"
		+ ((background != COLOR_NONE)?("\033[" + std::to_string (40 + background) + "m"):"")
		+ text
		+ "\033[0;m"
  ;
}

void print_sorts(std::map<std::string,std::set<std::string>> sorts) {
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
}