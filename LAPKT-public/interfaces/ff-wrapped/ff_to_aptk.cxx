/*
Lightweight Automated Planning Toolkit
Copyright (C) 2012
Miquel Ramirez <miquel.ramirez@rmit.edu.au>
Nir Lipovetzky <nirlipo@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ff_to_aptk.hxx>
#include <action.hxx>
#include <negation_action.hxx>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <map>



namespace aptk
{

namespace  FF_Parser {

void	get_problem_description( std::string pddl_domain_path,
					std::string pddl_problem_path,
					STRIPS_Problem& strips_problem,
					bool ignore_action_costs,
					bool get_detailed_fluent_names )
{
	FF_reinitialize_globals();
	FF_parse_problem( pddl_domain_path.c_str(), pddl_problem_path.c_str() );
	//	std::cout << "FF-preprocessing of PDDL problem description" << std::endl;
	FF_instantiate_problem();
	//	std::cout << "Facts in problem:" << gnum_ft_conn << std::endl;

	strips_problem.set_domain_name( FF::get_domain_name() );
	strips_problem.set_problem_name( FF::get_problem_name() );


    std::vector<unsigned > args;


	for ( int i = 0; i < gnum_ft_conn; i++ )
	{
		if ( !get_detailed_fluent_names )
		{
			std::string ft_name = FF::get_ft_name(i);
#if 0
            /** chao edit
             *
             */
//            std::pair<std::string,std::vector<unsigned>> ft_member = FF::get_ft_name_edit(i,args);
//            std::string ft_name=ft_member.first;
//            std::vector<unsigned> constants_vector =ft_member.second;
#endif

            /** chao edit
             *
             */
			STRIPS_Problem::add_fluent( strips_problem, ft_name);
            //STRIPS_Problem::add_fluent_edit( strips_problem, ft_name,constants_vector);
			continue;
		}
		std::string ft_signature = FF::get_ft_name(i);
		STRIPS_Problem::add_fluent( strips_problem, ft_signature  );
	}
    	Fluent_Vec I, G;
	FF::get_initial_state( I );
	FF::get_goal_state( G );
    Fluent_Ptr_Vec q=strips_problem.fluents();

#ifdef DEGUB
    for ( int i = 0; i < gnum_ft_conn; i++ )
    {
        if ( !get_detailed_fluent_names )
        {

            std::pair<std::string,std::vector<unsigned>> ft_member = FF::get_ft_name_edit(i,args);
            std::string ft_name=ft_member.first;
            STRIPS_Problem::add_fluent_negation( strips_problem, ft_name);
            std::stringstream buffer;
            std::string ft_negation_name=ft_member.first;
            buffer<<"(not-"<<ft_negation_name<<")";
            aptk::STRIPS_Problem::add_fluent_negation(strips_problem,buffer.str());

            continue;
        }
        std::string ft_signature = FF::get_ft_name(i);
        STRIPS_Problem::add_fluent( strips_problem, ft_signature  );
    }
#endif

#ifdef DEBUG
	/** chao edit add the constants
	 *
	 */
//
//	Fluent_Vec conFluents;
//    Fluent_Set conFluentsSet;
//    Fluent_Set conSet;
//    Fluent_Vec conIndexs;
//    std::map <unsigned , unsigned > con_dic;
//    conFluentsSet.resize(strips_problem.fluents().size()+I.size());
//    conSet.resize(strips_problem.fluents().size()+I.size());
//
//    for (unsigned p : I){
//
//        if (std::find(G.begin(), G.end(), p) != G.end()){
//            continue;
//        } else{
//            conFluents.push_back(p);
//        }
//    }
//
//    for (unsigned p: conFluents ){
//        conFluentsSet.set(p);
//        std::stringstream buffer;
//        buffer<<"(con-"<<strips_problem.fluents()[p]->signature()<<")";
//        unsigned  fl_idx = aptk::STRIPS_Problem::add_fluent(strips_problem,buffer.str());
//        con_dic.insert(std::pair<int, int>(p, fl_idx));
//        conSet.set(fl_idx);
//        conIndexs.push_back(fl_idx);
//        G.push_back(fl_idx);
//    }

#endif
#ifdef  DEGUB
    /** chao add the negation
     *
     */

//	Fluent_Vec negFluents;
//	Fluent_Set negFluentsSet;
//	Fluent_Vec negIndexs;
////	std::set<unsigned > goal_constants;
//
//
//    std::map <unsigned , unsigned > dic;
//    negFluentsSet.resize(strips_problem.fluents().size()+G.size()+I.size());
//
//	for (unsigned p : G){
////	    for (unsigned c : strips_problem.fluents()[p]->constants()){
////	        goal_constants.insert(c);
////	    }
////        if (conSet.isset(p)){
////            continue;
////        }
//        if (std::find(I.begin(), I.end(), p) != I.end()){
//            continue;
//        } else{
//            negFluents.push_back(p);
//        }
//    }
//	for (unsigned p: negFluents ){
//	    negFluentsSet.set(p);
//	    std::stringstream buffer;
//	    buffer<<"(not-"<<strips_problem.fluents()[p]->signature()<<")";
//	    unsigned  fl_idx = aptk::STRIPS_Problem::add_fluent(strips_problem,buffer.str());
//	    negIndexs.push_back(fl_idx);
//        dic.insert(std::pair<int, int>(p, fl_idx));
//	    I.push_back(fl_idx);
//
//	}


//	bool object_flag= true;
//	for (unsigned p: I){
//	    if (strips_problem.fluents()[p]->constants().empty())
//        {
//	        object_flag=false;
//        }
//        for(unsigned c :strips_problem.fluents()[p]->constants()){
//          if (goal_constants.find(c)!=goal_constants.end())
//          {
//              object_flag= false;
//              break;
//          }if the flunet occured as the inital sate and doesn't occured in the goal state,
//
//        }
//        if (object_flag )
//        {
//            G.push_back(p);
//        }
//	}


#endif

//    STRIPS_Problem::set_negation( strips_problem, negIndexs);
	STRIPS_Problem::set_init( strips_problem, I);
	STRIPS_Problem::set_goal( strips_problem, G);





	//	std::cout << "Operators in problem:" << gnum_ef_conn << std::endl;

        bool with_costs = false;
        for ( int i = 0; i < gnum_ef_conn; i++ )
                if( FF::get_op_metric_cost( i ) != 0 )
                {
                        with_costs = true;
                        break;
                }

	if ( ignore_action_costs ) with_costs = false;

        if(gconditional_effects)
	{
		for ( int i = 0; i < gnum_op_conn; i++ )
		{
			if( ! (gop_conn[i].action) )
				continue;

			std::string op_name = FF::get_op_name( gop_conn[i].action );
			
			Fluent_Vec op_precs;

			for( int j = 0; j < gop_conn[i].action->num_preconds; j++)
				op_precs.push_back( gop_conn[i].action->preconds[j] );
			
			bool inconsistent = false;
			float op_cost = 0;
			Conditional_Effect_Vec cond_effects;
			for( int j = 0; j < gop_conn[i].num_E; j++)
			{
				unsigned ef = gop_conn[i].E[j];

				if ( gef_conn[ef].removed == TRUE ) continue;
				if ( gef_conn[ef].illegal == TRUE ) continue;

			
				Fluent_Vec  op_conds, op_adds, op_dels;

				//Do not add the preconditions of the action into the CE
				int common_prec = gop_conn[i].action->num_preconds;
				
				for ( int j = common_prec; j < gef_conn[ef].num_PC; j++ ){				
					op_conds.push_back( gef_conn[ef].PC[j] );
				}
				for ( int j = 0; j < gef_conn[ef].num_A; j++ )
					op_adds.push_back( gef_conn[ef].A[j] );
				for ( int j = 0; j < gef_conn[ef].num_D; j++ )
					op_dels.push_back( gef_conn[ef].D[j] );

				
				if(with_costs)
				{
					if ( gef_conn[ef].num_IN == 0 ) {
						op_cost = 0;
					}
					else if ( gef_conn[ef].num_IN >= 1 ) {
						op_cost = gef_conn[ef].cost;
					}
				}
				else
					op_cost = 1;
				for ( auto p : op_adds ) 
					if ( std::find( op_dels.begin(), op_dels.end(), p ) != op_dels.end() ) {
						inconsistent = true;
						break;
					}
				if ( inconsistent ) {
					std::cout << "WARNING: FF Parser: Operator " << op_name << " has inconsistent adds and deletes in conditional effect #" << ef+1 << std::endl;
					break;
				}
	
				for ( auto p : op_dels ) 
					if ( std::find( op_adds.begin(), op_adds.end(), p ) != op_adds.end() ) {
						inconsistent = true;
						break;
					}
				if ( inconsistent ) {
					std::cout << "WARNING: FF Parser: Operator " << op_name << " has inconsistent adds and deletes in conditional effect #" << ef+1 << std::endl;
					break;
				}
		
				Conditional_Effect* new_cef = new Conditional_Effect( strips_problem );
				new_cef->define( op_conds, op_adds, op_dels );
				cond_effects.push_back( new_cef );
			}
			
			if ( inconsistent ) {
				continue;
			}

			unsigned op_idx;
			Fluent_Vec op_adds, op_dels;

			op_idx = STRIPS_Problem::add_action( strips_problem, op_name, op_precs, op_adds, op_dels, cond_effects );
			strips_problem.actions()[op_idx]->set_cost( op_cost );
			
		}
	}
	else
	{
		for ( int i = 0; i < gnum_ef_conn; i++ )
		{
			if ( gef_conn[i].removed == TRUE ) continue;
			if ( gef_conn[i].illegal == TRUE ) continue;

			std::string op_name = FF::get_op_name(i);
			Fluent_Vec  op_precs, op_adds, op_dels;
#ifdef DEGUB
            Fluent_Vec  op_negation_precs, op_negation_adds, op_negation_dels;
#endif
			Conditional_Effect_Vec cond_effects;


			for ( int j = 0; j < gef_conn[i].num_PC; j++ ){
                op_precs.push_back( gef_conn[i].PC[j] );
//                op_negation_precs.push_back( gef_conn[i].PC[j]*2 );
			}

			for ( int j = 0; j < gef_conn[i].num_A; j++ )
            {
                op_adds.push_back( gef_conn[i].A[j] );
//                op_negation_adds.push_back( gef_conn[i].A[j]*2 );
            }

			for ( int j = 0; j < gef_conn[i].num_D; j++ )
            {
                op_dels.push_back( gef_conn[i].D[j] );
//                op_negation_dels.push_back( gef_conn[i].D[j]*2 );
            }

#ifdef  DEGUB
/** chao add for negation actions
 *
 */
//            for ( int j = 0; j < op_negation_adds.size(); j++ ){
//                if (negFluentsSet.isset(op_adds[j])){
//                    op_dels.push_back(dic[op_adds[j]]);
//                    op_precs.push_back(dic[op_adds[j]]);
//                }
//            }
//            for ( int j = 0; j < op_negation_adds.size(); j++ ){
//                    op_negation_dels.push_back(strips_problem.negation_fluents()[j*2+1]->index());
//                    op_negation_precs.push_back(dic[op_adds[j]]);
//                }
//
#endif
#ifdef  DEGUB
            /** chao edit for neation action
             *
             */
            for ( int j = 0; j < op_negation_adds.size(); j++ ){
                 {
                    op_negation_dels.push_back(op_negation_adds[j]+1);
                    op_negation_precs.push_back(op_negation_adds[j]+1);
                }
            }
            for ( int j = 0; j < op_negation_dels.size(); j++ ){
                if (op_negation_dels[j]%2==0)
                {
                    op_negation_adds.push_back(op_negation_dels[j]+1);
                }
            }
#endif

#ifdef  DEGUB
            /** chao add the negation
             *
             */

//			for ( int j = 0; j < op_adds.size(); j++ ){
//                if (negFluentsSet.isset(op_adds[j])){
//                    op_dels.push_back(dic[op_adds[j]]);
//                    op_precs.push_back(dic[op_adds[j]]);
//                }
//            }
//
//            for ( int j = 0; j < op_dels.size(); j++ ){
//                if (negFluentsSet.isset(op_dels[j])){
//                    op_adds.push_back(dic[op_dels[j]]);
//                }
//            }

            /** chao edit
              *  if the flunet occured as the inital sate and doesn't occured in the goal state, this flunets should be added to the add list
              */
//            for ( int j = 0; j < op_precs.size(); j++ ){
//                if (std::find(op_dels.begin(), op_dels.end(), op_precs[j]) != op_dels.end()){
//                    continue;
//                } else{
//                    op_adds.push_back(op_precs[j]);
//                }
//            }


            /** chao add the constants
              *
              */

//            for ( int j = 0; j < op_adds.size(); j++ ){
//                if (conFluentsSet.isset(op_adds[j])){
//                    op_adds.push_back(con_dic[op_adds[j]]);
//                }
//            }
//
//            for ( int j = 0; j < op_dels.size(); j++ ){
//                if (conFluentsSet.isset(op_dels[j]) and std::find(op_adds.begin(), op_adds.end(), con_dic[op_dels[j]]) == op_adds.end()){
//                    op_adds.push_back(con_dic[op_dels[j]]);
//                }
//            }

#endif
            float op_cost = 0;
			if(with_costs)
			{
				if ( gef_conn[i].num_IN == 0 ) {
					op_cost = 0;
				}
				else if ( gef_conn[i].num_IN >= 1 ) {
					op_cost = gef_conn[i].cost;
				}
			}
			else
				op_cost = 1;

			bool inconsistent = false;
			for ( auto p : op_adds ) 
				if ( std::find( op_dels.begin(), op_dels.end(), p ) != op_dels.end() ) {
					inconsistent = true;
					break;
				}
			if ( inconsistent ) {
				std::cout << "WARNING: FF Parser: Operator " << op_name << " has inconsistent adds and deletes" << std::endl;
				continue;
			}

			for ( auto p : op_dels ) 
				if ( std::find( op_adds.begin(), op_adds.end(), p ) != op_adds.end() ) {
					inconsistent = true;
					break;
				}
			if ( inconsistent ) {
				std::cout << "WARNING: FF Parser: Operator " << op_name << " has inconsistent adds and deletes" << std::endl;
				continue;
			}

			unsigned op_idx;
			op_idx = STRIPS_Problem::add_action( strips_problem, op_name, op_precs, op_adds, op_dels, cond_effects );
			strips_problem.actions()[op_idx]->set_cost( op_cost );
#ifdef DEGUB
            unsigned op_negation_idx;
            op_negation_idx = STRIPS_Problem::add_negation_action( strips_problem, op_name, op_negation_precs, op_negation_adds, op_negation_dels, cond_effects );
            strips_problem.negation_actions()[op_negation_idx]->set_cost( op_cost );
#endif
		}

#ifdef DEGUB
		/** chao add to negation
		 *
		 */


		/** chao add to make constant action
		 *
		 */

//		for (unsigned p: conFluents){
//            std::stringstream buffer;
//            buffer<<"--"<< strips_problem.fluents()[p]->signature()<<"--"<<strips_problem.fluents()[con_dic[p]]->signature()<<".";
//
//            Fluent_Vec  op_precs, op_adds, op_dels,cond_effects;
//            Conditional_Effect_Vec cond_effects_edit;
//
//
//            op_precs.push_back( p );
//
//            op_adds.push_back( con_dic[p]);
//            float op_cost = 0;
//
//            unsigned op_idx;
//            op_idx = STRIPS_Problem::add_action( strips_problem, buffer.str(), op_precs, op_adds, op_dels, cond_effects_edit );
//            strips_problem.actions()[op_idx]->set_cost( op_cost );
//
//
//
//        }

#endif
	}

	strips_problem.make_action_tables();
#ifdef DEGUG
    strips_problem.make_negation_action_tables(false);
#endif
	strips_problem.make_effect_tables();
#ifdef DEBUG
	strips_problem.make_negation_effect_tables();
#endif
	
}

}

}
