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

#include <iostream>
#include <fstream>

/**
 * LAPKT includes
 */
#include <ff_to_aptk.hxx>
#include <strips_prob.hxx>
#include <fluent.hxx>
#include <action.hxx>
#include <cond_eff.hxx>
#include <strips_state.hxx>
#include <bwd_search_prob.hxx>
#include <fwd_search_prob.hxx>

#include <novelty_partition.hxx>
#include <landmark_graph.hxx>
#include <landmark_graph_generator.hxx>
#include <landmark_graph_manager.hxx>
#include <h_2.hxx>
#include <h_1.hxx>
#include <layered_h_max.hxx>


#include <aptk/open_list.hxx>
#include <aptk/string_conversions.hxx>

#include <boost/program_options.hpp>


/**
 * Local includes
 */

//NIR: Files adapted from LAPKT
#include "novelty_partition_2.hxx"
#include "ff_rp_heuristic.hxx"
#include "rp_heuristic_bfws.hxx"
#include "landmark_count.hxx"
#include <novelty.hxx>
#include "bfws_forward_backward.hxx"
#include "bfws_4h.hxx"
#include "bfws_2h.hxx"
#include "bfws_2h_M.hxx"
#include "bfws_2h_consistency.hxx"
#include "bfws_2h_consistency_M.hxx"




#include "new_node_comparer.hxx"

namespace po = boost::program_options;

/**
 * NAMESPACES 
 */


//NIR: Model 
using	aptk::STRIPS_Problem;
using	aptk::agnostic::bwd_Search_Problem;
using	aptk::agnostic::Fwd_Search_Problem;
using	aptk::Action;


//NIR: Heuristics
using 	aptk::agnostic::Landmarks_Graph;
using 	aptk::agnostic::Landmarks_Graph_Generator;
using   aptk::agnostic::Landmarks_Graph_Manager;
using 	aptk::agnostic::Landmarks_Count_Heuristic;

using 	aptk::agnostic::H2_Heuristic;
using 	aptk::agnostic::H1_Heuristic;
using   aptk::agnostic::Layered_H_Max;

using	aptk::agnostic::H_Add_Evaluation_Function;
using	aptk::agnostic::H_Max_Evaluation_Function;
using	aptk::agnostic::Relaxed_Plan_Heuristic;
using   aptk::agnostic::FF_Relaxed_Plan_Heuristic;


//NIR: Novelties

using 	aptk::agnostic::Novelty;

using 	aptk::agnostic::Novelty_Partition;
using 	aptk::agnostic::Novelty_Partition_2;


//NIR: Open List and evaluation functions
using 	aptk::search::Open_List;
using	aptk::search::Node_Comparer_4H;
using	aptk::search::Node_Comparer_2H;


//NIR: Search Engines
using aptk::search::bfws_forward_backward::BFWS_Forward_Backward;
using	aptk::search::bfws_4h::BFWS_4H;
using	aptk::search::bfws_2h::BFWS_2H;
using	aptk::search::bfws_2h::BFWS_2H_M;
using	aptk::search::bfws_2h::BFWS_2H_Consistency;
using	aptk::search::bfws_2h::BFWS_2H_Consistency_M;
/**
 * DEFINITIONS
 */

//NIR: Heuristics

typedef         H2_Heuristic<bwd_Search_Problem,Fwd_Search_Problem>                  H2_Fwd;
typedef		    aptk::search::bfws_forward_backward::Node< bwd_Search_Problem, Fwd_Search_Problem,aptk::State >	          bfws_Node;

typedef         Landmarks_Graph_Generator<bwd_Search_Problem, Fwd_Search_Problem>     Gen_Lms;
/** chao edit
 *
 */

typedef         Landmarks_Graph_Generator<Fwd_Search_Problem,Fwd_Search_Problem>     Gen_Lms_Fwd_Forward;

typedef         Landmarks_Count_Heuristic<bwd_Search_Problem,Fwd_Search_Problem>     H_Lmcount_Fwd;

typedef         Landmarks_Graph_Manager<bwd_Search_Problem,Fwd_Search_Problem>       Land_Graph_Man;

typedef         Landmarks_Graph_Manager<bwd_Search_Problem,Fwd_Search_Problem>       Land_Graph_Man_Backward;

/** chao edit
 *
 */
typedef         Landmarks_Graph_Manager<bwd_Search_Problem,Fwd_Search_Problem>       Land_Graph_Man_Forward;



// NIR: Node representations for each search algorithm
typedef		aptk::search::bfws_4h::Node< bwd_Search_Problem, Fwd_Search_Problem,aptk::State >	Search_Node_4h;
//typedef		aptk::search::bfws_2h::Node< bwd_Search_Problem,Fwd_Search_Problem, aptk::State >	Search_Node_2h;

typedef		aptk::search::bfws_forward_backward::Node< bwd_Search_Problem,Fwd_Search_Problem, aptk::State >	Search_Node_2h_Forward_Backward;

// NIR: Novelty functions for each node type. Novelty partition expects class
// node to define partition() function. '_2' version expects partition2() function.  
typedef         Novelty_Partition<bwd_Search_Problem, Fwd_Search_Problem, Search_Node_4h>               H_Novel_Fwd_4h;
typedef         Novelty_Partition_2<bwd_Search_Problem, Fwd_Search_Problem,Search_Node_4h>             H_Novel_2_Fwd_4h;

//typedef         Novelty_Partition<bwd_Search_Problem, Fwd_Search_Problem,Search_Node_2h>               H_Novel_Fwd_2h;

typedef         Novelty_Partition<bwd_Search_Problem, Fwd_Search_Problem,Search_Node_2h_Forward_Backward>               H_Novel_Fwd_2h_Forward_backward;

// NIR: Then we define the type of the tie-breaking algorithm
// for the open list we are going to use
typedef		Node_Comparer_4H< Search_Node_4h >				Tie_Breaking_Algorithm_4h;
//typedef		Node_Comparer_2H< Search_Node_2h >	        		Tie_Breaking_Algorithm_2h;

typedef		Node_Comparer_2H< Search_Node_2h_Forward_Backward >	        		Tie_Breaking_Algorithm_2h_Forward_Backward;


// NIR: Now we define the Open List type by combining the types we have defined before
typedef		Open_List< Tie_Breaking_Algorithm_4h, Search_Node_4h >	          	        BFS_Open_List_4h;
//typedef		Open_List< Tie_Breaking_Algorithm_2h, Search_Node_2h >	                        BFS_Open_List_2h;

typedef		Open_List< Tie_Breaking_Algorithm_2h_Forward_Backward , Search_Node_2h_Forward_Backward >	                        BFS_Open_List_2h_Forward_Backward;

// NIR: Now we define the heuristics

/**chao edit
 *  change the 	H_Add_Fwd (bwd to fwd)
 */

typedef		H1_Heuristic<bwd_Search_Problem, Fwd_Search_Problem, H_Add_Evaluation_Function>	H_Add_Fwd;
typedef		Relaxed_Plan_Heuristic< bwd_Search_Problem, Fwd_Search_Problem, H_Add_Fwd >		H_Add_Rp_Fwd;

//typedef		Relaxed_Plan_Heuristic< Fwd_Search_Problem, H_Add_Fwd >		H_Add_Rp_Fwd_Forward;

typedef         Layered_H_Max< bwd_Search_Problem >				      Alt_H_Max;
typedef         FF_Relaxed_Plan_Heuristic< bwd_Search_Problem, Alt_H_Max, unsigned >     Classic_FF_H_Max;

// NIR: Now we're ready to define the BFS algorithm we're going to use, H_Lmcount can be used only with goals,
// or with landmarks computed from s0
// typedef	BFWS_2H< bwd_Search_Problem,Fwd_Search_Problem, H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd,  BFS_Open_List_2h>                       k_BFWS;

typedef	BFWS_Forward_Backward< bwd_Search_Problem, Fwd_Search_Problem, H_Novel_Fwd_2h_Forward_backward , H_Lmcount_Fwd, H_Add_Rp_Fwd, BFS_Open_List_2h_Forward_Backward>                       k_FFWS;



 //typedef	BFWS_2H_M< bwd_Search_Problem, Fwd_Search_Problem,H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd,  BFS_Open_List_2h >                    k_BFWS_M;
 typedef        BFWS_4H< bwd_Search_Problem,Fwd_Search_Problem, H_Novel_Fwd_4h, H_Lmcount_Fwd, H_Novel_2_Fwd_4h, H_Add_Rp_Fwd,  BFS_Open_List_4h >    BFWS_w_hlm_hadd;
typedef	BFWS_2H_M< bwd_Search_Problem, Fwd_Search_Problem,H_Novel_Fwd_2h_Forward_backward , H_Lmcount_Fwd, H_Add_Rp_Fwd,  BFS_Open_List_2h_Forward_Backward >                    k_BFWS_M;
// NIR: Consistency Search variants
 //typedef	BFWS_2H_Consistency< bwd_Search_Problem,Fwd_Search_Problem,H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd,  BFS_Open_List_2h >    k_BFWS_Consistency;
 //typedef	BFWS_2H_Consistency_M< bwd_Search_Problem,Fwd_Search_Problem,H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd, BFS_Open_List_2h >   k_BFWS_Consistency_M;

typedef	BFWS_2H_Consistency< bwd_Search_Problem,Fwd_Search_Problem,H_Novel_Fwd_2h_Forward_backward , H_Lmcount_Fwd, H_Add_Rp_Fwd,  BFS_Open_List_2h_Forward_Backward >    k_BFWS_Consistency;
typedef	BFWS_2H_Consistency_M< bwd_Search_Problem,Fwd_Search_Problem,H_Novel_Fwd_2h_Forward_backward , H_Lmcount_Fwd, H_Add_Rp_Fwd, BFS_Open_List_2h_Forward_Backward >   k_BFWS_Consistency_M;



template <typename Search_Engine>
void bfws_options( bwd_Search_Problem&	search_prob_bwd, Landmarks_Graph& graph_bwd,Fwd_Search_Problem& search_prob_fwd,Landmarks_Graph& graph_fwd,Search_Engine& bfs_engine, unsigned& max_novelty){

    /**
     * check later
     */
	bfs_engine.set_max_novelty_bwd( max_novelty );
	bfs_engine.set_use_novelty_bwd( true );
    bfs_engine.set_max_novelty_fwd( max_novelty );
    bfs_engine.set_use_novelty_fwd( true );
	/** check later
	 *
	 */
	bfs_engine.rel_fl_h().ignore_rp_h_value_bwd(true);
    bfs_engine.rel_fl_h().ignore_rp_h_value_fwd(true);

	//NIR: engine doesn't own the pointer, need to free at the end
	Land_Graph_Man* lgm = new Land_Graph_Man( search_prob_bwd, &graph_bwd,search_prob_fwd, &graph_fwd);
    /**
     * check later
     */
	bfs_engine.use_land_graph_manager( lgm );
//    bfs_engine.use_land_graph_manager_fwd( lgm );

	//NIR: Approximate the domain of #r counter, so we can initialize the novelty table, making sure we've got
	//     space for novelty > 1 tuples 
	H_Add_Rp_Fwd hadd( search_prob_bwd,search_prob_fwd);
	float h_init_fwd=0;
    float h_init_bwd=0;
	/** chao edit
	 * change the search_prob.init_state() to search_prob,goal_state()
	 */
	const aptk::State* s_0_fwd = search_prob_fwd.init_state();
    const aptk::State* s_0_bwd = search_prob_bwd.goal_state();
	hadd.eval_fwd( *s_0_fwd, h_init_fwd );
    hadd.eval_bwd( *s_0_bwd, h_init_bwd );
   /** check later
    *
    */

	bfs_engine.set_arity_bwd( max_novelty, graph_bwd.num_landmarks()*h_init_bwd );
    bfs_engine.set_arity_fwd( max_novelty, graph_fwd.num_landmarks()*h_init_fwd );

	
}

template <typename Search_Engine>
float do_search( Search_Engine& engine, STRIPS_Problem& plan_prob, std::ofstream& plan_stream, bool &found_plan, Gen_Lms* gen_lms=NULL) {

	std::ofstream	details( "execution.details" );
    /**
     * check later
     */
	engine.start_bwd();
    engine.start_fwd();

	std::vector< aptk::Action_Idx > plan;
	float				cost_bwd = -1;
    float				cost_fwd = -1;

	float ref = aptk::time_used();
	float t0 = aptk::time_used();

    /**
     * check later
     */
	unsigned expanded_0_bwd = engine.expanded_bwd();
	unsigned generated_0_bwd = engine.generated_bwd();

    unsigned expanded_0_fwd = engine.expanded_fwd();
    unsigned generated_0_fwd = engine.generated_fwd();


	found_plan = engine.find_solution( cost_bwd, cost_fwd,plan,gen_lms );

	
	if ( found_plan  ) {
		details << "Forward Plan found with cost: " << cost_fwd << std::endl;
        details << "Backward Plan found with cost: " << cost_bwd << std::endl;
		for ( unsigned k = 0; k < plan.size(); k++ ) {
			details << k+1 << ". ";
			const aptk::Action& a = *(plan_prob.actions()[ plan[k] ]);
			details << a.signature();
			details << std::endl;
			plan_stream << a.signature() << std::endl;
		}
		float tf = aptk::time_used();
        /**
     * check later
     */
		unsigned expanded_b = engine.expanded_bwd();
		unsigned generated_b = engine.generated_bwd();

        unsigned expanded_f = engine.expanded_fwd();
        unsigned generated_f = engine.generated_fwd();
		details << "Time: " << tf - t0 << std::endl;
		/**
		 * check later
		 */

        details << "Forkward Generated: " << generated_f - generated_0_fwd << std::endl;
        details << "Forward Expanded: " << expanded_f - expanded_0_fwd << std::endl;

		details << "Backward Generated: " << generated_b - generated_0_bwd << std::endl;
		details << "Backward Expanded: " << expanded_b - expanded_0_bwd << std::endl;

		t0 = tf;
		/**
		 * check later
		 */
		expanded_0_fwd = expanded_f;
		generated_0_fwd = generated_f;

        expanded_0_bwd = expanded_b;
        generated_0_bwd = generated_b;
		plan.clear();

		float total_time = aptk::time_used() - ref;
		std::cout << "Total time: " << total_time << std::endl;
        /**
     * check later
     */
		std::cout << "Forward Nodes generated during search: " << engine.generated_fwd() << std::endl;
		std::cout << "Forward Nodes expanded during search: " << engine.expanded_fwd() << std::endl;

        std::cout << "Backward Nodes generated during search: " << engine.generated_bwd() << std::endl;
        std::cout << "Backward Nodes expanded during search: " << engine.expanded_bwd() << std::endl;

        std::cout << "Forward Plan found with cost: " << cost_fwd << std::endl;
		std::cout << "Backward Plan found with cost: " << cost_bwd << std::endl;
		details.close();
		return total_time;

	}
	else{
		float tf = aptk::time_used();
        /**
     * check later
     */
		unsigned expanded_f = engine.expanded_fwd();
		unsigned generated_f = engine.generated_fwd();

        unsigned expanded_b = engine.expanded_bwd();
        unsigned generated_b = engine.generated_bwd();

		details << "Time: " << tf - t0 << std::endl;
		/**
		 * check later
		 */
		details << "Forward Generated: " << generated_f - generated_0_fwd << std::endl;
		details << "Forward Expanded: " << expanded_f - expanded_0_fwd << std::endl;

        details << "Backward Generated: " << generated_b - generated_0_bwd << std::endl;
        details << "Backward Expanded: " << expanded_b - expanded_0_bwd << std::endl;
		t0 = tf;
		/**
		 * check later
		 */
		expanded_0_fwd = expanded_f;
		generated_0_fwd = generated_f;

        expanded_0_bwd = expanded_b;
        generated_0_bwd = generated_b;

	 	float total_time = aptk::time_used() - ref;	
		std::cout << "Total time: " << total_time << std::endl;
        /**
     * check later
     */
		std::cout << "Forward Nodes generated during search: " << engine.generated_fwd() << std::endl;
		std::cout << "Forward Nodes expanded during search: " << engine.expanded_fwd() << std::endl;

        std::cout << "Backward Nodes generated during search: " << engine.generated_bwd() << std::endl;
        std::cout << "Backward Nodes expanded during search: " << engine.expanded_bwd() << std::endl;
		std::cout << "Plan found with cost: NOTFOUND" << std::endl;
		details.close();
		return total_time;
	}				      
}

template <typename Search_Engine>
float do_search_forward( Search_Engine& engine, STRIPS_Problem& plan_prob, std::ofstream& plan_stream, bool &found_plan, Gen_Lms* gen_lms=NULL ) {

    std::ofstream	details( "execution.details" );
    /**
     * check later
     */
    engine.start_fwd();

    std::vector< aptk::Action_Idx > plan;
    float				cost_fwd = -1;

    float ref = aptk::time_used();
    float t0 = aptk::time_used();

    /**
     * check later
     */

    unsigned expanded_0_fwd = engine.expanded_fwd();
    unsigned generated_0_fwd = engine.generated_fwd();


    found_plan = engine.find_solution_forward( cost_fwd,plan,gen_lms );


    if ( found_plan  ) {
        details << "Forward Plan found with cost: " << cost_fwd << std::endl;
        for ( unsigned k = 0; k < plan.size(); k++ ) {
            details << k+1 << ". ";
            const aptk::Action& a = *(plan_prob.actions()[ plan[k] ]);
            details << a.signature();
            details << std::endl;
            plan_stream << a.signature() << std::endl;
        }
        float tf = aptk::time_used();
        /**
     * check later
     */

        unsigned expanded_f = engine.expanded_fwd();
        unsigned generated_f = engine.generated_fwd();
        details << "Time: " << tf - t0 << std::endl;
        /**
         * check later
         */

        details << "Forkward Generated: " << generated_f - generated_0_fwd << std::endl;
        details << "Forward Expanded: " << expanded_f - expanded_0_fwd << std::endl;


        t0 = tf;
        /**
         * check later
         */
        expanded_0_fwd = expanded_f;
        generated_0_fwd = generated_f;

        plan.clear();

        float total_time = aptk::time_used() - ref;
        std::cout << "Total time: " << total_time << std::endl;
        /**
     * check later
     */
        std::cout << "Forward Nodes generated during search: " << engine.generated_fwd() << std::endl;
        std::cout << "Forward Nodes expanded during search: " << engine.expanded_fwd() << std::endl;


        std::cout << "Forward Plan found with cost: " << cost_fwd << std::endl;
        details.close();
        return total_time;

    }
    else{
        float tf = aptk::time_used();
        /**
     * check later
     */
        unsigned expanded_f = engine.expanded_fwd();
        unsigned generated_f = engine.generated_fwd();


        details << "Time: " << tf - t0 << std::endl;
        /**
         * check later
         */
        details << "Forward Generated: " << generated_f - generated_0_fwd << std::endl;
        details << "Forward Expanded: " << expanded_f - expanded_0_fwd << std::endl;

        t0 = tf;
        /**
         * check later
         */
        expanded_0_fwd = expanded_f;
        generated_0_fwd = generated_f;

        float total_time = aptk::time_used() - ref;
        std::cout << "Total time: " << total_time << std::endl;
        /**
     * check later
     */
        std::cout << "Forward Nodes generated during search: " << engine.generated_fwd() << std::endl;
        std::cout << "Forward Nodes expanded during search: " << engine.expanded_fwd() << std::endl;

        std::cout << "Plan found with cost: NOTFOUND" << std::endl;
        details.close();
        return total_time;
    }
}

template <typename Search_Engine>
float do_search_backward( Search_Engine& engine, STRIPS_Problem& plan_prob, std::ofstream& plan_stream, bool &found_plan, Gen_Lms* gen_lms=NULL ) {

    std::ofstream	details( "execution.details" );
    /**
     * check later
     */
    engine.start_bwd();

    std::vector< aptk::Action_Idx > plan;
    float				cost_bwd = -1;

    float ref = aptk::time_used();
    float t0 = aptk::time_used();

    /**
     * check later
     */
    unsigned expanded_0_bwd = engine.expanded_bwd();
    unsigned generated_0_bwd = engine.generated_bwd();



    found_plan = engine.find_solution_backward( cost_bwd,plan,gen_lms );


    if ( found_plan  ) {
        details << "Backward Plan found with cost: " << cost_bwd << std::endl;
        for ( unsigned k = 0; k < plan.size(); k++ ) {
            details << k+1 << ". ";
            const aptk::Action& a = *(plan_prob.actions()[ plan[k] ]);
            details << a.signature();
            details << std::endl;
            plan_stream << a.signature() << std::endl;
        }
        float tf = aptk::time_used();
        /**
     * check later
     */
        unsigned expanded_b = engine.expanded_bwd();
        unsigned generated_b = engine.generated_bwd();

        details << "Time: " << tf - t0 << std::endl;
        /**
         * check later
         */


        details << "Backward Generated: " << generated_b - generated_0_bwd << std::endl;
        details << "Backward Expanded: " << expanded_b - expanded_0_bwd << std::endl;

        t0 = tf;
        /**
         * check later
         */

        expanded_0_bwd = expanded_b;
        generated_0_bwd = generated_b;
        plan.clear();

        float total_time = aptk::time_used() - ref;
        std::cout << "Total time: " << total_time << std::endl;
        /**
     * check later
     */

        std::cout << "Backward Nodes generated during search: " << engine.generated_bwd() << std::endl;
        std::cout << "Backward Nodes expanded during search: " << engine.expanded_bwd() << std::endl;

        std::cout << "Backward Plan found with cost: " << cost_bwd << std::endl;
        details.close();
        return total_time;

    }
    else{
        float tf = aptk::time_used();
        /**
     * check later
     */

        unsigned expanded_b = engine.expanded_bwd();
        unsigned generated_b = engine.generated_bwd();

        details << "Time: " << tf - t0 << std::endl;
        /**
         * check later
         */
        details << "Backward Generated: " << generated_b - generated_0_bwd << std::endl;
        details << "Backward Expanded: " << expanded_b - expanded_0_bwd << std::endl;
        t0 = tf;
        /**
         * check later
         */

        expanded_0_bwd = expanded_b;
        generated_0_bwd = generated_b;

        float total_time = aptk::time_used() - ref;
        std::cout << "Total time: " << total_time << std::endl;
        /**
     * check later
     */

        std::cout << "Backward Nodes generated during search: " << engine.generated_bwd() << std::endl;
        std::cout << "Backward Nodes expanded during search: " << engine.expanded_bwd() << std::endl;
        std::cout << "Plan found with cost: NOTFOUND" << std::endl;
        details.close();
        return total_time;
    }
}
void process_command_line_options( int ac, char** av, po::variables_map& vars ) {
	po::options_description desc( "Options:",135,260 );
	
	desc.add_options()
		( "help", "Show help message. " )
		( "domain", po::value<std::string>(), "Input PDDL domain description" )
		( "problem", po::value<std::string>(), "Input PDDL problem description" )
		( "output", po::value<std::string>(), "Output file for plan" )
		( "max_novelty", po::value<int>()->default_value(2), "Max width w for novelty (default 2)")
		( "ignore_costs", po::value<bool>()->default_value(true), "Ignore action costs")
		;
	po::options_description desc_search( "Search Algorithms:",135,260 );
	
	desc_search.add_options()
		( "DUAL-FB", po::value<bool>()->default_value(false), "forward-backward first with novelty 1, then BFWS using h_ff and h_landcount as in AAAI-17 paper")
		( "BFWS-f5", po::value<bool>()->default_value(false),
  		        "BFWS(w,#g), w_{#r,#g}, as in BFWS(f5) AAAI-17 paper" )
            ( "BFWS-f5-backward", po::value<bool>()->default_value(false))

		;

	po::options_description desc_poly_search( "Polynomial Search Algorithms:",135,260 );
	
	desc_poly_search.add_options()
		( "k-BFWS", po::value<bool>()->default_value(false),
			"k-BFWS(w,#g), w_{#r,#g}, pruning w > k, where k = bound() argument, default 2" )
            ( "k-BFWS-backward", po::value<bool>()->default_value(false))
            ( "k-BDWS-e", po::value<bool>()->default_value(false))
            ( "k-BDWS-e-head", po::value<bool>()->default_value(false))
            ( "k-BDWS-e-close", po::value<bool>()->default_value(false))
            ( "k-BDWS-e-random", po::value<bool>()->default_value(false))
            ( "k-BDWS-f-head", po::value<bool>()->default_value(false))
            ( "k-BDWS-f-close", po::value<bool>()->default_value(false))
            ( "forward-backward", po::value<bool>()->default_value(false), "k-BFWS first, then k-BFWS-backward")

		;

	po::options_description all_desc;
	all_desc.add(desc).add(desc_search).add(desc_poly_search);
	try {
		po::store( po::parse_command_line( ac, av, all_desc ), vars );	       
		po::notify( vars );
	}
	catch ( std::exception& e ) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::exit(1);
	}
	catch ( ... ) {
		std::cerr << "Exception of unknown type!" << std::endl;
		std::exit(1);
	}

	if ( vars.count("help") ) {
		std::cout << all_desc << std::endl;
		std::exit(0);
	}

}

void report_no_solution( std::string reason, std::ofstream& plan_stream ) {
	plan_stream << ";; No solution found" << std::endl;
	plan_stream << ";; " << reason << std::endl;
	plan_stream.close();
}

int main( int argc, char** argv ) {

	po::variables_map vm;

	process_command_line_options( argc, argv, vm );

	
	if ( !vm.count( "domain" ) ) {
		std::cerr << "No PDDL domain was specified!" << std::endl;
		std::exit(1);
	}

	if ( !vm.count( "problem" ) ) {
		std::cerr << "No PDDL problem was specified!" << std::endl;
		std::exit(1);
	}

	std::ofstream	plan_stream;
	
	if ( !vm.count( "output" ) ) {
		std::cerr << "No output plan file specified, defaulting to 'plan.ipc'" << std::endl;
		plan_stream.open( "plan.ipc" );
	}
	else
		plan_stream.open( vm["output"].as<std::string>() );


	STRIPS_Problem	prob;
	bool ignore_costs = vm["ignore_costs"].as<bool>();


	aptk::FF_Parser::get_problem_description( vm["domain"].as<std::string>(), vm["problem"].as<std::string>(), prob, ignore_costs  );
	std::cout << "PDDL problem description loaded: " << std::endl;
	std::cout << "\tDomain: " << prob.domain_name() << std::endl;
	std::cout << "\tProblem: " << prob.problem_name() << std::endl;
	std::cout << "\t#Actions: " << prob.num_actions() << std::endl;
	std::cout << "\t#Fluents: " << prob.num_fluents() << std::endl;


    /**
     *  compute the mutexes
     */
	bwd_Search_Problem	search_prob( &prob );
	Fwd_Search_Problem  fwd_search_prob (&prob);
	if (!prob.has_conditional_effects()){
	    auto* h2 =new H2_Fwd (search_prob,fwd_search_prob);
	    h2->compute_edeletes_bwd(prob);
	    search_prob.set_h2_bwd(h2);
	} else
	    prob.compute_edeletes();

# if 0
    /**
     * goal closure for backward search
     **/

    aptk::Fluent_chao m_relevant_fluents;
    aptk::Fluent_Vec  m_new_init;
    aptk::Fluent_Vec  goal_record;
    m_relevant_fluents.resize(prob.num_fluents() );

    for ( unsigned i = 0; i < prob.num_actions(); i++ ) {

        auto a = (prob.actions()[i]);

        // Relevant if the fluent is in the precondition
        for ( unsigned j = 0; j < a->add_vec().size(); ++j ) {
            // only allow 1 action to add relevant fluents. TRY min h2.value_op(i)
            //if( !m_relevant_fluents[a->add_vec()[j]].empty() ) continue;
            for (unsigned k = 0; k < a->add_vec().size(); ++k) {
                if ( a->add_vec()[j]!= a->add_vec()[k] ){
                    m_relevant_fluents[a->add_vec()[j]].push_back(a->add_vec()[k]);

                    }
            }
//            for (unsigned k = 0; k < a->prec_vec().size(); ++k) {
//                unsigned prec = a->prec_vec()[k];
//                if ( a->add_vec()[j]!= prec && !a->del_set().isset(prec) ){
//                    m_relevant_fluents[a->add_vec()[j]].push_back(prec);
//
//                }
//            }
        }
    }
    m_new_init=prob.goal();
    goal_record=prob.goal();
    for (unsigned p : goal_record){
        for (unsigned q: m_relevant_fluents[p]){
            bool goal_flag=TRUE;
            if (std::find(m_new_init.begin(), m_new_init.end(), q) != m_new_init.end()) {
                continue;
            }
            else{
                for (unsigned w: m_new_init){
                    if (search_prob.h2_fwd().is_mutex(w,q)){
                        goal_flag=FALSE;
                        break;
                    }
                }
                //if (goal_flag)
                //{
                //	for (unsigned e: prob.init()){
                //        if (search_prob.h2_fwd().is_mutex(e,q)){
                //            init_flag=FALSE;
                //            break;
                //        }
                //    }
                //}
            }
         if (goal_flag){
             if (std::find(m_new_init.begin(), m_new_init.end(), q) != m_new_init.end()) {
                 continue;
             }
             prob.goal().push_back(q);
             m_new_init.push_back(q);
         }
        }
    }
    // add the fluents to our goal state. if want to ignore added fluents, just comment this.
    STRIPS_Problem::set_goal( prob, m_new_init);

#endif


    /** build the I-ordering**/
	Gen_Lms    gen_lms( search_prob,fwd_search_prob);
	Landmarks_Graph graph_bwd( prob );
    Landmarks_Graph graph_fwd( prob );
	gen_lms.set_only_goals_fwd(true );
	gen_lms.set_goal_ordering_fwd(true );
    gen_lms.set_only_goals_bwd(true );
    gen_lms.set_goal_ordering_bwd(true );


    /**
     * check the compute_lm_graph_set_additive_forward_fwd suitable for graph_bwd and the same for backward
     */
    gen_lms.compute_lm_graph_set_additive_fwd(graph_fwd);

	gen_lms.compute_lm_graph_set_additive_bwd( graph_bwd );


	bool found_plan = false;
	
	unsigned max_novelty  = vm["max_novelty"].as<int>();


    if(vm["BFWS-f5"].as<bool>()){

        std::cout << "Starting search with BFWS-f5" << std::endl;

        k_FFWS bfs_engine( search_prob ,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        float bfs_t = do_search_forward( bfs_engine, prob, plan_stream, found_plan,&gen_lms );

        std::cout << "Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

        return 0;

    }else if(vm["k-BFWS"].as<bool>()){
		
		std::cout << "Starting search with k-BFWS..." << std::endl;
			
		//k_BFWS bfs_engine( search_prob);
        k_FFWS bfs_engine(search_prob,fwd_search_prob);

		bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

		bfs_engine.set_use_novelty_pruning_fwd( true );
		
		float bfs_t = do_search_forward( bfs_engine, prob, plan_stream, found_plan,&gen_lms  );

		std::cout << "Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

		plan_stream.close();

		return 0;
    } else if (vm["BFWS-f5-backward"].as<bool>()){
        std::cout << "Starting search with BFWS-f5-backward" << std::endl;

        k_FFWS bfs_engine( search_prob ,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        float bfs_t = do_search_backward( bfs_engine, prob, plan_stream, found_plan,&gen_lms );

        std::cout << "backward Fast-BFS search completed in " << bfs_t << " secs" << std::endl;


    } else if (vm["k-BFWS-backward"].as<bool>()){
        std::cout << "Starting search with k-BFWS-backward..." << std::endl;
        //k_BFWS bfs_engine( search_prob);
        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        bfs_engine.set_use_novelty_pruning_bwd( true );

        float bfs_t = do_search_backward( bfs_engine, prob, plan_stream, found_plan,&gen_lms  );

        std::cout << "backward Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

        plan_stream.close();

        return 0;

    }else if (vm["k-BDWS-e"].as<bool>()){

        std::cout << "Starting search with k-BDWS-e..." << std::endl;
        prob.set_direction("e");
        prob.set_intersection("novelty");

        //k_BFWS bfs_engine( search_prob);
        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        bfs_engine.set_use_novelty_pruning_bwd( true );
        bfs_engine.set_use_novelty_pruning_fwd( true );

        float bfs_t = do_search( bfs_engine, prob, plan_stream, found_plan,&gen_lms);

        std::cout << "Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

        plan_stream.close();

        return 0;

    }else if (vm["k-BDWS-e-head"].as<bool>()){

        std::cout << "Starting search with k-BDWS-e-head..." << std::endl;
        prob.set_direction("e");
        prob.set_intersection("head");

        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        bfs_engine.set_use_novelty_pruning_bwd( true );
        bfs_engine.set_use_novelty_pruning_fwd( true );

        float bfs_t = do_search( bfs_engine, prob, plan_stream, found_plan,&gen_lms);

        std::cout << "Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

        plan_stream.close();

        return 0;

    }else if (vm["k-BDWS-e-close"].as<bool>()){


        std::cout << "Starting search with k-BDWS-e-close..." << std::endl;
        prob.set_direction("e");
        prob.set_intersection("close");

        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        bfs_engine.set_use_novelty_pruning_bwd( true );
        bfs_engine.set_use_novelty_pruning_fwd( true );

        float bfs_t = do_search( bfs_engine, prob, plan_stream, found_plan,&gen_lms );

        std::cout << "Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

        plan_stream.close();

        return 0;

    } else if (vm["k-BDWS-e-random"].as<bool>()){

        std::cout << "Starting search with k-BDWS-e-random..." << std::endl;
        prob.set_direction("e");
        prob.set_intersection("random");

        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        bfs_engine.set_use_novelty_pruning_bwd( true );
        bfs_engine.set_use_novelty_pruning_fwd( true );

        float bfs_t = do_search( bfs_engine, prob, plan_stream, found_plan,&gen_lms );

        std::cout << "Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

        plan_stream.close();

        return 0;
    }
    else if (vm["k-BDWS-f-head"].as<bool>()){

        std::cout << "Starting search with k-BDWS-f-head..." << std::endl;
        prob.set_direction("f");
        prob.set_intersection("head");


        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        bfs_engine.set_use_novelty_pruning_bwd( true );
        bfs_engine.set_use_novelty_pruning_fwd( true );

        float bfs_t = do_search( bfs_engine, prob, plan_stream, found_plan,&gen_lms );

        std::cout << "Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

        plan_stream.close();

        return 0;
    }
    else if (vm["k-BDWS-f-close"].as<bool>()){

        std::cout << "Starting search with k-BDWS-f-close..." << std::endl;
        prob.set_direction("f");
        prob.set_intersection("close");

        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );

        bfs_engine.set_use_novelty_pruning_bwd( true );
        bfs_engine.set_use_novelty_pruning_fwd( true );

        float bfs_t = do_search( bfs_engine, prob, plan_stream, found_plan,&gen_lms );

        std::cout << "Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

        plan_stream.close();

        return 0;
    }






    if( vm["forward-backward"].as<bool>() && !found_plan)	{

        std::cout << "Starting forward search with k-BFWS-forward..." << std::endl;

        //k_BFWS bfs_engine( search_prob);
        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        /** check later
         *
         */
        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );
        /**
         * check later
         *
         */
        bfs_engine.set_use_novelty_pruning_fwd( true );

        float bfs_t = do_search_forward( bfs_engine, prob, plan_stream, found_plan,&gen_lms  );

        std::cout << "forward Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

    }


    if(!found_plan && vm["forward-backward"].as<bool>()  ){
        std::cout << "Starting backward search with k-BFWS-backward..." << std::endl;



        //k_BFWS bfs_engine( search_prob);
        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        /**
         * Use landmark count instead of goal count
         */

        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );
        /**
         * check later
         *
         */

        bfs_engine.set_use_novelty_pruning_bwd( true );

        float bfs_t = do_search_backward( bfs_engine, prob, plan_stream, found_plan,&gen_lms  );

        std::cout << "backward BFS search completed in " << bfs_t << " secs" << std::endl;

    }


    if( vm["DUAL-FB"].as<bool>() and !found_plan)	{

        std::cout << "Starting forward search with k-BFWS..." << std::endl;

        //k_BFWS bfs_engine( search_prob);
        k_FFWS bfs_engine(search_prob,fwd_search_prob);
        max_novelty = 1;

        /** check later
         *
         */
        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );
        /**
         * check later
         *
         */
        bfs_engine.set_use_novelty_pruning_fwd( true );

        float bfs_t = do_search_forward( bfs_engine, prob, plan_stream, found_plan,&gen_lms  );

        std::cout << "forward Fast-BFS search completed in " << bfs_t << " secs" << std::endl;

    }


    if(!found_plan && vm["DUAL-FB"].as<bool>()  ){
        std::cout << "Starting backward search with k-BFWS..." << std::endl;



        //k_BFWS bfs_engine( search_prob);
        k_FFWS bfs_engine(search_prob,fwd_search_prob);

        /**
         * Use landmark count instead of goal count
         */
        max_novelty = 1;
        bfws_options( search_prob,graph_bwd,fwd_search_prob,graph_fwd,bfs_engine, max_novelty );
        /**
         * check later
         *
         */
        bfs_engine.set_use_novelty_pruning_bwd( true );

        float bfs_t = do_search_backward( bfs_engine, prob, plan_stream, found_plan,&gen_lms  );

        std::cout << "backward BFS search completed in " << bfs_t << " secs" << std::endl;

    }

    if(!found_plan && vm["DUAL-FB"].as<bool>() ){
        std::cout << "Starting search with BFWS(novel,land,h_ff)..." << std::endl;

        BFWS_w_hlm_hadd bfs_engine( search_prob, fwd_search_prob );
        bfs_engine.h4().ignore_rp_h_value_fwd(true);

        /**
         * Use landmark count instead of goal count
         */
        Gen_Lms    gen_lms( search_prob, fwd_search_prob );
        gen_lms.set_only_goals_fwd( false );
        Landmarks_Graph graph1( prob );
        Landmarks_Graph graph2( prob );
        gen_lms.compute_lm_graph_set_additive_fwd( graph1 );

        Land_Graph_Man lgm( search_prob,&graph2, fwd_search_prob, &graph1);
        bfs_engine.use_land_graph_manager( &lgm );

        std::cout << "Landmarks found: " << graph1.num_landmarks() << std::endl;
        std::cout << "Landmarks_Edges found: " << graph1.num_landmarks_and_edges() << std::endl;

        max_novelty = vm["max_novelty"].as<int>();

        bfs_engine.set_arity( max_novelty, graph1.num_landmarks_and_edges() );

        //NIR: Approximate the domain of #r counter, so we can initialize the novelty table, making sure we've got
        //     space for novelty > 1 tuples
        H_Add_Rp_Fwd hadd( search_prob, fwd_search_prob);
        float h_init=0;
        const aptk::State* s_0 = fwd_search_prob.init_state();
        hadd.eval_fwd( *s_0, h_init );

        bfs_engine.set_arity_2( max_novelty,  h_init );

        found_plan = false;
        float bfs_t = do_search_forward( bfs_engine, prob, plan_stream, found_plan );

        std::cout << "BFS search completed in " << bfs_t << " secs" << std::endl;

    }


    plan_stream.close();

	return 0;
}
