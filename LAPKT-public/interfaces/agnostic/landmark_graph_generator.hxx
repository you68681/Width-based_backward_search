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

#ifndef __LANDMARK_GRAPH_GENERATOR__
#define __LANDMARK_GRAPH_GENERATOR__

#include <aptk/search_prob.hxx>
#include <aptk/heuristic.hxx>
#include <reachability.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <landmark_graph.hxx>
#include <h_1.hxx>
#include <action.hxx>
#include <vector>
#include <deque>
#include <iosfwd>

namespace aptk {

namespace agnostic {



template <typename bwd_Search_Problem, typename Fwd_Search_Problem >
class Landmarks_Graph_Generator  {

typedef		H1_Heuristic<bwd_Search_Problem, Fwd_Search_Problem, H_Max_Evaluation_Function>	H_Max;

public:

    /**
     * solution 1 original use the prob.task() but this is the. changed to prob, in order to record the swap of initial state and goal state.
     * change the m_strips_mode const type STRIPS_Problem& to Search_Model&
     * solution 2. change all m_strips_model.init() to prob.init(). doing same with goal()
     */

	Landmarks_Graph_Generator( const bwd_Search_Problem& prob_bwd, const Fwd_Search_Problem& prob_fwd)
	:  m_strips_model_bwd( prob_bwd.task() ),  m_strips_model_fwd( prob_fwd.task() ),m_only_goals_bwd( false ), m_only_goals_fwd( false ),m_goal_ordering_bwd(true), m_goal_ordering_fwd(true),m_h1( prob_bwd,prob_fwd),m_verbose_bwd( false ), m_verbose_fwd( false ),
	m_collect_lm_in_init_bwd(false), m_collect_lm_in_init_fwd(false),search_prob_bwd(prob_bwd),search_prob_fwd(prob_fwd)
	{

		m_reachability_bwd = new aptk::agnostic::Reachability_Test( prob_bwd.task() );
        m_reachability_fwd = new aptk::agnostic::Reachability_Test( prob_fwd.task() );
	}

	virtual ~Landmarks_Graph_Generator() {
		delete m_reachability_bwd;
        delete m_reachability_fwd;
	}


	void	set_verbose_bwd( bool v ) { m_verbose_bwd = v; }
    void	set_verbose_fwd( bool v ) { m_verbose_bwd = v; }
	
public:

	void	allow_lm_in_init_bwd() { m_collect_lm_in_init_bwd = true; }
	void	disallow_lm_in_init_bwd() { m_collect_lm_in_init_bwd = false; }

	void   set_only_goals_bwd( bool b ){ m_only_goals_bwd = b; }

	void   set_goal_ordering_bwd( bool b ){ m_goal_ordering_bwd= b; }

    void	allow_lm_in_init_fwd() { m_collect_lm_in_init_fwd = true; }
    void	disallow_lm_in_init_fwd() { m_collect_lm_in_init_fwd = false; }

    void   set_only_goals_fwd( bool b ){ m_only_goals_fwd = b; }

    void   set_goal_ordering_fwd( bool b ){ m_goal_ordering_fwd = b; }
#ifdef DEBUG
    void    build_goal_ordering_forward_bwd( Landmarks_Graph& graph ){

        for ( unsigned i = 0; i < m_strips_model_bwd.goal().size(); i++ ) {
            unsigned p = m_strips_model_bwd.goal()[i];
            for ( unsigned j = i+1; j < m_strips_model_bwd.goal().size(); j++ ) {
                unsigned q = m_strips_model_bwd.goal()[j];

                /**
                 * If all actions adding p edel q, then p must precede q
                 */
                const std::vector<const Action*>& add_acts_p = m_strips_model_bwd.actions_adding( p );

                bool all_actions_edel_q = true;
                for ( unsigned k = 0; k < add_acts_p.size(); k++ ) {
                    //add_acts_p[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(p)->signature() << " edel " << m_strips_model.fluents().at(q)->signature() << "? " <<std::endl;
                    if( ! add_acts_p[k]->edel_set().isset( q ) ){
                        all_actions_edel_q = false;
                        break;
                    }

                }
                /**
                 * If all actions adding q edel p, then q must precede p
                 */

                const std::vector<const Action*>& add_acts_q = m_strips_model_bwd.actions_adding( q );
                bool all_actions_edel_p = true;
                for ( unsigned k = 0; k < add_acts_q.size(); k++ ) {
                    //add_acts_q[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(q)->signature() << " edel " << m_strips_model.fluents().at(p)->signature() << "? " <<std::endl;
                    if( ! add_acts_q[k]->edel_set().isset( p ) ){
                        all_actions_edel_p = false;
                        break;
                    }
                }
                /**
                 * Avoid loops in the graph
                 */
                if(all_actions_edel_q &&  all_actions_edel_p)continue;

                if(all_actions_edel_q){
                    graph.add_landmark_for( q, p );
//					continue;
                }
                if(all_actions_edel_p){
                    graph.add_landmark_for( p, q );
                }


            }


        }

    }
#endif

#ifdef DEBUG
    void    build_goal_ordering_forward_fwd( Landmarks_Graph& graph ){

        for ( unsigned i = 0; i < m_strips_model_fwd.goal().size(); i++ ) {
            unsigned p = m_strips_model_fwd.goal()[i];
            for ( unsigned j = i+1; j < m_strips_model_fwd.goal().size(); j++ ) {
                unsigned q = m_strips_model_fwd.goal()[j];

                /**
                 * If all actions adding p edel q, then p must precede q
                 */
                const std::vector<const Action*>& add_acts_p = m_strips_model_fwd.actions_adding( p );

                bool all_actions_edel_q = true;
                for ( unsigned k = 0; k < add_acts_p.size(); k++ ) {
                    //add_acts_p[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(p)->signature() << " edel " << m_strips_model.fluents().at(q)->signature() << "? " <<std::endl;
                    if( ! add_acts_p[k]->edel_set().isset( q ) ){
                        all_actions_edel_q = false;
                        break;
                    }

                }
                /**
                 * If all actions adding q edel p, then q must precede p
                 */

                const std::vector<const Action*>& add_acts_q = m_strips_model_fwd.actions_adding( q );
                bool all_actions_edel_p = true;
                for ( unsigned k = 0; k < add_acts_q.size(); k++ ) {
                    //add_acts_q[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(q)->signature() << " edel " << m_strips_model.fluents().at(p)->signature() << "? " <<std::endl;
                    if( ! add_acts_q[k]->edel_set().isset( p ) ){
                        all_actions_edel_p = false;
                        break;
                    }
                }
                /**
                 * Avoid loops in the graph
                 */
                if(all_actions_edel_q &&  all_actions_edel_p)continue;

                if(all_actions_edel_q){
                    graph.add_landmark_for( q, p );
//					continue;
                }
                if(all_actions_edel_p){
                    graph.add_landmark_for( p, q );
                }


            }


        }

    }
#endif
  
        void    build_goal_ordering_bwd( Landmarks_Graph& graph ,State *fwd_state=NULL ){
        /**
         * change the m_strips_model.goal() to prob.gaol()
         */
            Fluent_Vec fwd_vec;
            if (fwd_state!=NULL){
                fwd_vec=fwd_state->fluent_vec();
            }
            else{
                fwd_vec=search_prob_bwd.goal();
            }
		//for ( unsigned i = 0; i < m_strips_model.goal().size(); i++ ) {
		//	unsigned p = m_strips_model.goal()[i];
		//	for ( unsigned j = i+1; j < m_strips_model.goal().size(); j++ ) {
		//		unsigned q = m_strips_model.goal()[j];

		for ( unsigned i = 0; i < fwd_vec.size(); i++ ) {

		    unsigned p = fwd_vec[i];
            //if (this->m_strips_model.get_negation()[p]) continue;
		    for ( unsigned j = i+1; j < fwd_vec.size(); j++ ) {
		        unsigned q = fwd_vec[j];
                //if (this->m_strips_model.get_negation()[q]) continue;


				/**
				 * If all actions adding p edel q, then p must precede q
				 */

				/** chao edit
				 *
				 */
                const std::vector<const Action*>& add_acts_p = m_strips_model_bwd.actions_requiring(p) ;
				//const std::vector<const Action*>& add_acts_p = m_strips_model.actions_adding( p );
				
				bool all_actions_edel_q = true;
				for ( unsigned k = 0; k < add_acts_p.size(); k++ ) {
					//add_acts_p[k]->print( m_strips_model, std::cout );
					//std::cout << m_strips_model.fluents().at(p)->signature() << " edel " << m_strips_model.fluents().at(q)->signature() << "? " <<std::endl;
					//if( ! add_acts_p[k]->edel_set().isset( q ) ){
                    /** chao edit
                     *  regression
                    */
                    if( ! add_acts_p[k]->bwd_edel_set().isset( q ) ){
						all_actions_edel_q = false;
						break;
					}      

				}
				/**
				 * If all actions adding q edel p, then q must precede p
				 */
				/** chao edit
				 *
				 */
                const std::vector<const Action*>& add_acts_q = m_strips_model_bwd.actions_requiring(q);
				//const std::vector<const Action*>& add_acts_q = m_strips_model.actions_adding( q );
				bool all_actions_edel_p = true;
				for ( unsigned k = 0; k < add_acts_q.size(); k++ ) {
					//add_acts_q[k]->print( m_strips_model, std::cout );
					//std::cout << m_strips_model.fluents().at(q)->signature() << " edel " << m_strips_model.fluents().at(p)->signature() << "? " <<std::endl;
					//if( ! add_acts_q[k]->edel_set().isset( p ) ){
					/** chao edit
					 *  rgression
					 */
                    if( ! add_acts_q[k]->bwd_edel_set().isset( p ) ){
                        all_actions_edel_p = false;
						break;
					}      
				}
				/**
				 * Avoid loops in the graph
				 */
				if(all_actions_edel_q &&  all_actions_edel_p)continue;

				if(all_actions_edel_q){
					graph.add_landmark_for( p, q );
					//continue;
				}
				if(all_actions_edel_p){
					graph.add_landmark_for( q, p );
				}


			}
				
	
		}

	}
    void    build_goal_ordering_fwd( Landmarks_Graph& graph,State *bwd_state=NULL ){
	    Fluent_Vec bwd_vec;
	    if (bwd_state!=NULL){
	        bwd_vec=bwd_state->fluent_vec();
	    }
	    else{
	        bwd_vec=m_strips_model_fwd.goal();
	    }

        for ( unsigned i = 0; i < bwd_vec.size(); i++ ) {
            unsigned p = bwd_vec[i];
            for ( unsigned j = i+1; j < bwd_vec.size(); j++ ) {
                unsigned q = bwd_vec[j];

                /**
                 * If all actions adding p edel q, then p must precede q
                 */
                const std::vector<const Action*>& add_acts_p = m_strips_model_fwd.actions_adding( p );

                bool all_actions_edel_q = true;
                for ( unsigned k = 0; k < add_acts_p.size(); k++ ) {
                    //add_acts_p[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(p)->signature() << " edel " << m_strips_model.fluents().at(q)->signature() << "? " <<std::endl;
                    if( ! add_acts_p[k]->edel_set().isset( q ) ){
                        all_actions_edel_q = false;
                        break;
                    }

                }
                /**
                 * If all actions adding q edel p, then q must precede p
                 */

                const std::vector<const Action*>& add_acts_q = m_strips_model_fwd.actions_adding( q );
                bool all_actions_edel_p = true;
                for ( unsigned k = 0; k < add_acts_q.size(); k++ ) {
                    //add_acts_q[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(q)->signature() << " edel " << m_strips_model.fluents().at(p)->signature() << "? " <<std::endl;
                    if( ! add_acts_q[k]->edel_set().isset( p ) ){
                        all_actions_edel_p = false;
                        break;
                    }
                }
                /**
                 * Avoid loops in the graph
                 */
                if(all_actions_edel_q &&  all_actions_edel_p)continue;

                if(all_actions_edel_q){
                    graph.add_landmark_for( q, p );
                    //continue;
                }
                if(all_actions_edel_p){
                    graph.add_landmark_for( p, q );
                }


            }


        }

    }



#ifdef DEBUG
    void    build_init_ordering_bwd( Landmarks_Graph& graph ){
        /**
         * change the m_strips_model.goal() to prob.gaol()
         */

        //for ( unsigned i = 0; i < m_strips_model.goal().size(); i++ ) {
        //	unsigned p = m_strips_model.goal()[i];
        //	for ( unsigned j = i+1; j < m_strips_model.goal().size(); j++ ) {
        //		unsigned q = m_strips_model.goal()[j];

        for ( Fluent_Vec::const_iterator it = m_strips_model_bwd.init().begin();
              it != m_strips_model_bwd.init().end(); it++ ) {
            Landmarks_Graph::Node* q_node=graph.node(unsigned (*it));
            if (q_node == NULL ){
                graph.add_landmark( *it );
            }
        }

        for ( unsigned i = 0; i < search_prob_bwd.goal().size(); i++ ) {

            unsigned p = search_prob_bwd.goal()[i];
            //if (this->m_strips_model.get_negation()[p]) continue;
            for ( unsigned j = i+1; j < search_prob_bwd.goal().size(); j++ ) {
                unsigned q = search_prob_bwd.goal()[j];
                //if (this->m_strips_model.get_negation()[q]) continue;


                /**
                 * If all actions adding p edel q, then p must precede q
                 */

                /** chao edit
                 *
                 */
                const std::vector<const Action*>& add_acts_p = m_strips_model_bwd.actions_requiring(p) ;
                //const std::vector<const Action*>& add_acts_p = m_strips_model.actions_adding( p );

                bool all_actions_edel_q = true;
                for ( unsigned k = 0; k < add_acts_p.size(); k++ ) {
                    //add_acts_p[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(p)->signature() << " edel " << m_strips_model.fluents().at(q)->signature() << "? " <<std::endl;
                    //if( ! add_acts_p[k]->edel_set().isset( q ) ){
                    /** chao edit
                     *  regression
                    */
                    if( ! add_acts_p[k]->bwd_edel_set().isset( q ) ){
                        all_actions_edel_q = false;
                        break;
                    }

                }
                /**
                 * If all actions adding q edel p, then q must precede p
                 */
                /** chao edit
                 *
                 */
                const std::vector<const Action*>& add_acts_q = m_strips_model_bwd.actions_requiring(q);
                //const std::vector<const Action*>& add_acts_q = m_strips_model.actions_adding( q );
                bool all_actions_edel_p = true;
                for ( unsigned k = 0; k < add_acts_q.size(); k++ ) {
                    //add_acts_q[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(q)->signature() << " edel " << m_strips_model.fluents().at(p)->signature() << "? " <<std::endl;
                    //if( ! add_acts_q[k]->edel_set().isset( p ) ){
                    /** chao edit
                     *  rgression
                     */
                    if( ! add_acts_q[k]->bwd_edel_set().isset( p ) ){
                        all_actions_edel_p = false;
                        break;
                    }
                }
                /**
                 * Avoid loops in the graph
                 */
                if(all_actions_edel_q &&  all_actions_edel_p)continue;

//                if(all_actions_edel_q){
//                    graph.add_landmark_for( q, p );
//                    //continue;
//                }
//                if(all_actions_edel_p){
//                    graph.add_landmark_for( p, q );
//                }
                if(all_actions_edel_q){
                    graph.add_landmark_for( p, q );
                    //continue;
                }
                if(all_actions_edel_p){
                    graph.add_landmark_for( q, p );
                }


            }


        }

    }
#endif

#ifdef DEBUG
    void    build_init_ordering_fwd( Landmarks_Graph& graph ){
        /**
         * change the m_strips_model.goal() to prob.gaol()
         */

        //for ( unsigned i = 0; i < m_strips_model.goal().size(); i++ ) {
        //	unsigned p = m_strips_model.goal()[i];
        //	for ( unsigned j = i+1; j < m_strips_model.goal().size(); j++ ) {
        //		unsigned q = m_strips_model.goal()[j];

        for ( Fluent_Vec::const_iterator it = m_strips_model_fwd.init().begin();
              it != m_strips_model_fwd.init().end(); it++ ) {
            Landmarks_Graph::Node* q_node=graph.node(unsigned (*it));
            if (q_node == NULL ){
                graph.add_landmark( *it );
            }
        }

        for ( unsigned i = 0; i < search_prob_fwd.goal().size(); i++ ) {

            unsigned p = search_prob_fwd.goal()[i];
            //if (this->m_strips_model.get_negation()[p]) continue;
            for ( unsigned j = i+1; j < search_prob_fwd.goal().size(); j++ ) {
                unsigned q = search_prob_fwd.goal()[j];
                //if (this->m_strips_model.get_negation()[q]) continue;


                /**
                 * If all actions adding p edel q, then p must precede q
                 */

                /** chao edit
                 *
                 */
                const std::vector<const Action*>& add_acts_p = m_strips_model_fwd.actions_requiring(p) ;
                //const std::vector<const Action*>& add_acts_p = m_strips_model.actions_adding( p );

                bool all_actions_edel_q = true;
                for ( unsigned k = 0; k < add_acts_p.size(); k++ ) {
                    //add_acts_p[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(p)->signature() << " edel " << m_strips_model.fluents().at(q)->signature() << "? " <<std::endl;
                    //if( ! add_acts_p[k]->edel_set().isset( q ) ){
                    /** chao edit
                     *  regression
                    */
                    if( ! add_acts_p[k]->bwd_edel_set().isset( q ) ){
                        all_actions_edel_q = false;
                        break;
                    }

                }
                /**
                 * If all actions adding q edel p, then q must precede p
                 */
                /** chao edit
                 *
                 */
                const std::vector<const Action*>& add_acts_q = m_strips_model_fwd.actions_requiring(q);
                //const std::vector<const Action*>& add_acts_q = m_strips_model.actions_adding( q );
                bool all_actions_edel_p = true;
                for ( unsigned k = 0; k < add_acts_q.size(); k++ ) {
                    //add_acts_q[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(q)->signature() << " edel " << m_strips_model.fluents().at(p)->signature() << "? " <<std::endl;
                    //if( ! add_acts_q[k]->edel_set().isset( p ) ){
                    /** chao edit
                     *  rgression
                     */
                    if( ! add_acts_q[k]->bwd_edel_set().isset( p ) ){
                        all_actions_edel_p = false;
                        break;
                    }
                }
                /**
                 * Avoid loops in the graph
                 */
                if(all_actions_edel_q &&  all_actions_edel_p)continue;

//                if(all_actions_edel_q){
//                    graph.add_landmark_for( q, p );
//                    //continue;
//                }
//                if(all_actions_edel_p){
//                    graph.add_landmark_for( p, q );
//                }
                if(all_actions_edel_q){
                    graph.add_landmark_for( p, q );
                    //continue;
                }
                if(all_actions_edel_p){
                    graph.add_landmark_for( q, p );
                }


            }


        }

    }
#endif
#ifdef  DEBUG
    void    build_goal_ordering_original_bwd( Landmarks_Graph& graph ){

        for ( unsigned i = 0; i < m_strips_model_bwd.goal().size(); i++ ) {
            unsigned p = m_strips_model_bwd.goal()[i];
            for ( unsigned j = i+1; j < m_strips_model_bwd.goal().size(); j++ ) {
                unsigned q = m_strips_model_bwd.goal()[j];
//        for ( unsigned i = 0; i < m_strips_model.init().size(); i++ ) {
//            unsigned p = m_strips_model.init()[i];
//            for ( unsigned j = i+1; j < m_strips_model.init().size(); j++ ) {
//                unsigned q = m_strips_model.init()[j];

                /**
                 * If all actions adding p edel q, then p must precede q
                 */
                const std::vector<const Action*>& add_acts_p = m_strips_model_bwd.actions_adding( p );

                bool all_actions_edel_q = true;
                for ( unsigned k = 0; k < add_acts_p.size(); k++ ) {
                    //add_acts_p[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(p)->signature() << " edel " << m_strips_model.fluents().at(q)->signature() << "? " <<std::endl;
                    if( ! add_acts_p[k]->edel_set().isset( q ) ){
                        all_actions_edel_q = false;
                        break;
                    }

                }
                /**
                 * If all actions adding q edel p, then q must precede p
                 */

                const std::vector<const Action*>& add_acts_q = m_strips_model_bwd.actions_adding( q );
                bool all_actions_edel_p = true;
                for ( unsigned k = 0; k < add_acts_q.size(); k++ ) {
                    //add_acts_q[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(q)->signature() << " edel " << m_strips_model.fluents().at(p)->signature() << "? " <<std::endl;
                    if( ! add_acts_q[k]->edel_set().isset( p ) ){
                        all_actions_edel_p = false;
                        break;
                    }
                }
                /**
                 * Avoid loops in the graph
                 */
                if(all_actions_edel_q &&  all_actions_edel_p)continue;

                if(all_actions_edel_q){
                    graph.add_landmark_for( q, p );
                    //continue;
                }
                if(all_actions_edel_p){
                    graph.add_landmark_for( p, q );
                }


            }


        }

    }
#endif

#ifdef  DEGUB
    void    build_goal_ordering_original_fwd( Landmarks_Graph& graph ){

        for ( unsigned i = 0; i < m_strips_model_fwd.goal().size(); i++ ) {
            unsigned p = m_strips_model_fwd.goal()[i];
            for ( unsigned j = i+1; j < m_strips_model_fwd.goal().size(); j++ ) {
                unsigned q = m_strips_model_fwd.goal()[j];
//        for ( unsigned i = 0; i < m_strips_model.init().size(); i++ ) {
//            unsigned p = m_strips_model.init()[i];
//            for ( unsigned j = i+1; j < m_strips_model.init().size(); j++ ) {
//                unsigned q = m_strips_model.init()[j];

                /**
                 * If all actions adding p edel q, then p must precede q
                 */
                const std::vector<const Action*>& add_acts_p = m_strips_model_fwd.actions_adding( p );

                bool all_actions_edel_q = true;
                for ( unsigned k = 0; k < add_acts_p.size(); k++ ) {
                    //add_acts_p[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(p)->signature() << " edel " << m_strips_model.fluents().at(q)->signature() << "? " <<std::endl;
                    if( ! add_acts_p[k]->edel_set().isset( q ) ){
                        all_actions_edel_q = false;
                        break;
                    }

                }
                /**
                 * If all actions adding q edel p, then q must precede p
                 */

                const std::vector<const Action*>& add_acts_q = m_strips_model_fwd.actions_adding( q );
                bool all_actions_edel_p = true;
                for ( unsigned k = 0; k < add_acts_q.size(); k++ ) {
                    //add_acts_q[k]->print( m_strips_model, std::cout );
                    //std::cout << m_strips_model.fluents().at(q)->signature() << " edel " << m_strips_model.fluents().at(p)->signature() << "? " <<std::endl;
                    if( ! add_acts_q[k]->edel_set().isset( p ) ){
                        all_actions_edel_p = false;
                        break;
                    }
                }
                /**
                 * Avoid loops in the graph
                 */
                if(all_actions_edel_q &&  all_actions_edel_p)continue;

                if(all_actions_edel_q){
                    graph.add_landmark_for( q, p );
                    //continue;
                }
                if(all_actions_edel_p){
                    graph.add_landmark_for( p, q );
                }


            }


        }

    }
#endif
    void	compute_lm_graph_set_additive_bwd( Landmarks_Graph& graph, State* head_forward=NULL ) {
        Bit_Set lm_set( m_strips_model_bwd.num_fluents() );
        Bit_Set processed( m_strips_model_bwd.num_fluents() );



		std::deque<unsigned> updated;

		Fluent_Vec head_forward_vec;

		if (head_forward!=NULL){
		    head_forward_vec=head_forward->fluent_vec();
		    graph.remove_landmark();
		} else{
		    head_forward_vec=m_strips_model_bwd.init();

		}


		// 1. Insert goal atoms as landmarks
		/**
		 * change the m_strips_model.goal() to prob.gaol(), change the is_in_init to is_in_goal
		 */
		if (m_collect_lm_in_init_bwd){
            for ( Fluent_Vec::const_iterator it = m_strips_model_bwd.goal().begin();
                  it != m_strips_model_bwd.goal().end(); it++ ) {
                graph.add_landmark( *it );
                if ( ! m_strips_model_bwd.is_in_init( *it ) ) {
                    updated.push_back( *it );
                }
            }
		} else{
            for ( Fluent_Vec::const_iterator it = head_forward_vec.begin();
                  it != head_forward_vec.end(); it++ ) {
                graph.add_landmark( *it );
                if ( ! m_strips_model_bwd.is_in_goal( *it ) ) {
                    updated.push_back( *it );
                }
            }
		}


//        for ( Fluent_Vec::const_iterator it = search_prob.goal().begin();
//
//             it != search_prob.goal().end(); it++ ) {
//        	graph.add_landmark( *it );
//        	if ( ! m_strips_model.is_in_goal( *it ) ) {
//
//        		updated.push_back( *it );
//        	}
//        }

//        while ( !updated.empty() ) {
//            unsigned p = updated.front();
//            updated.pop_front();


//            const std::vector<const Action*>& add_acts = m_strips_model.actions_adding( p );
////            const std::vector<const Action*>& add_acts = m_strips_model.actions_requiring( p );
//            //std::cout << "Added by " << add_acts.size() << " actions" << std::endl;
//            if ( !add_acts.empty() ) {
//                lm_set.reset();
////                lm_set.add( add_acts[0]->add_set() );
//                lm_set.add( add_acts[0]->prec_set() );
//
//                for ( unsigned k = 1; k < add_acts.size(); k++ )
////                    lm_set.set_intersection( add_acts[k]->add_set() );
//                    lm_set.set_intersection( add_acts[k]->prec_set() );
//            }
//
//            for (unsigned q : lm_set ) {
//                if ( !m_collect_lm_in_init && m_strips_model.is_in_init(q) ) {
////                if ( !m_collect_lm_in_init && m_strips_model.is_in_goal(q) ) {
//                    continue;
//                }
//
//                if ( !graph.is_landmark(q) )
//                    graph.add_landmark( q );
//
//                graph.add_landmark_for( p, q );
//
//                updated.push_back( q );
//            }
//        }

		if( m_only_goals_bwd ){
			if(m_goal_ordering_bwd)
				build_goal_ordering_bwd( graph,head_forward);
			return;
		}
#if DEBUG
//		Bit_Set lm_set( m_strips_model.num_fluents() );
//		Bit_Set processed( m_strips_model.num_fluents() );

//
//        Bit_Set lm_set_chao( m_strips_model.num_fluents() );
//        Bit_Set processed_chao( m_strips_model.num_fluents() );
//
//        Bit_Set lm_set_chao_edl( m_strips_model.num_fluents() );
//        Bit_Set processed_chao_edl( m_strips_model.num_fluents() );

		while ( !updated.empty() ) {
			unsigned p = updated.front();
			updated.pop_front();

//			if ( processed_chao.isset(p) ) continue;
//			processed_chao.set(p);
//
//            if ( processed_chao_edl.isset(p) ) continue;
//            processed_chao_edl.set(p);

            /** original version
             *
             */
			//std::cout << "Processing landmark: " << m_strips_model.fluents()[ p ]->signature() << std::endl;
			const std::vector<const Action*>& add_acts = m_strips_model_bwd.actions_adding( p );
			//std::cout << "Added by " << add_acts.size() << " actions" << std::endl;
			if ( !add_acts.empty() ) {
				lm_set.reset();
				lm_set.add( add_acts[0]->prec_set() );

				for ( unsigned k = 1; k < add_acts.size(); k++ )
					lm_set.set_intersection( add_acts[k]->prec_set() );
			}

//            const std::vector<const Action*>& add_acts = m_strips_model.actions_requiring( p );
//			//std::cout << "Added by " << add_acts.size() << " actions" << std::endl;
//			if ( !add_acts.empty() ) {
//				lm_set.reset();
//				lm_set.add( add_acts[0]->add_set() );
//
//				for ( unsigned k = 1; k < add_acts.size(); k++ )
//					lm_set.set_intersection( add_acts[k]->add_set() );
//			}

//            if ( !add_acts.empty() ) {
//                lm_set_chao.reset();
//                lm_set_chao.add( add_acts[0]->del_set() );
//
//                for ( unsigned k = 1; k < add_acts.size(); k++ )
//                    lm_set_chao.set_intersection( add_acts[k]->del_set() );
//            }

//            if ( !add_acts.empty() ) {
//                lm_set_chao_edl.reset();
//                lm_set_chao_edl.add( add_acts[0]->edel_set() );
//
//                for ( unsigned k = 1; k < add_acts.size(); k++ )
//                    lm_set_chao_edl.set_intersection( add_acts[k]->edel_set() );
//            }

//            for (unsigned q : lm_set_chao ) {
//                if (p==q)continue;
//                graph.node(p)->add_del(graph.generate_del(q));
//                if ( graph.is_landmark(q) ){
//                    graph.add_landmark_for( q, p );
//                }
//
//            }

//            for (unsigned q : lm_set_chao_edl ) {
//                if (p==q)continue;
//                graph.node(p)->add_edel(graph.generate_edel(q));
//                if (  graph.is_landmark(q) ){
//                    graph.add_landmark_for(q,p);
//                }
//            }


//			const std::vector< std::pair< unsigned, const Action*> >& add_acts_ce =
//				m_strips_model.ceffs_adding( p );
//
//			if ( !add_acts_ce.empty() ) {
//
//				for ( unsigned k = 0; k < add_acts_ce.size(); k++ ) {
//					lm_set.set_intersection( add_acts_ce[k].second->prec_set() );
//					lm_set.set_intersection( add_acts_ce[k].second->ceff_vec()[ add_acts_ce[k].first ]->prec_set() );
//				}
//
//			}

			//std::cout << "LM set size: " << lm_set.bits().count_elements() << std::endl;
			/** change the is_in_init to is_in_goal()
			 *
			 */
			for (unsigned q : lm_set ) {
				if ( !m_collect_lm_in_init_bwd && m_strips_model_bwd.is_in_init(q) ) {
//				  if ( !m_collect_lm_in_init && m_strips_model.is_in_goal(q) ) {
					continue;
				}

				if ( !graph.is_landmark(q) )
					graph.add_landmark( q );
				/** original
				 *
				 */
				//				if ( ! m_strips_model.is_in_goal(p) )
//				graph.add_landmark_for( p, q );
               /** chao edit
                *
                */
                graph.add_landmark_for( q, p );

				// else{
				// 	graph.add_landmark_for( p, q );
				// 	//graph.node(p)->add_precedent_gn( graph.node(q) );
				// 	//graph.node(q)->add_requiring_gn( graph.node(p) );
				// }
				updated.push_back( q );
			}
		}


		/**
		 * Compute Greedy Necessary Orderings
		 */
		 /**
		  * change the  m_strips_model.init() to  prob.goal()
		  */
//        Fluent_Vec chao;
		Bit_Set reach_actions;
//		reach_actions.resize(search_prob.task().num_negation_actions());
//		for (int i=0; i<search_prob.task().negation_actions().size();i++){
//            auto a = search_prob.task().negation_actions()[i];
//		    for (unsigned p: search_prob.goal()){
//		        if (a->add_negation_set().isset(p*2)){
//		            reach_actions.set(a->index());
//		            chao.push_back(a->index());
//                    break;
//		        }
//		    }
//		}
		//m_reachability->get_reachable_actions( m_strips_model.init() , m_strips_model.goal() , reach_actions  );
		//float h_goal;
		//State init_s( m_strips_model );
		//init_s.set( m_strips_model.init() );
		//m_h1.eval( init_s, h_goal );
//		Fluent_Vec special_goal;
//            for ( unsigned k = 0; k < search_prob.task().negation_fluents().size(); k++ ){
//                unsigned index=search_prob.task().negation_fluents()[k]->index();
//                if ( index%2==0&& std::find(search_prob.goal().begin(), search_prob.goal().end(), index/2) != search_prob.goal().end()){
//                    special_goal.push_back(k);
//                    k++;
//                    continue;
//                } else{
//                    special_goal.push_back(k);
//                }
//		}
//        m_reachability->get_reachable_actions( search_prob.init() , search_prob.goal() , reach_actions  );
        m_reachability_bwd->get_reachable_actions_original( search_prob_bwd.goal(), search_prob_bwd.init(), reach_actions  );
//        m_reachability->get_reachable_negation_actions( search_prob.init() , search_prob.goal() , reach_actions  );
        float h_goal;
        State init_s( m_strips_model_bwd );
        /** chao edit
         *
         */
//        init_s.set( search_prob.init() );
        init_s.set( m_strips_model_bwd.init());
        m_h1.eval_bwd( init_s, h_goal );

		for(unsigned p = 1; p <  m_strips_model_bwd.num_fluents(); p++){

			if( ! graph.is_landmark(p) ) continue;

//            if( std::find(search_prob.init().begin(), search_prob.init().end(), p)==search_prob.init().end() ) continue;
			Action_Ptr_Const_Vec best_supp;

			m_h1.get_best_supporters_bwd( p, best_supp );

			//const std::vector<const Action*>& add_acts = m_strips_model.actions_adding( p );

			lm_set.reset();

			for ( unsigned k = 0; k < best_supp.size(); k++ ){
				const Action* a = best_supp[k];
				/**
				 * if action is reachable
				 */
				if( !reach_actions.isset(a->index()) ) continue;

				Bit_Set lands_a( m_strips_model_bwd.num_fluents() );

				getPCFluentLandmarks_bwd( a->index(), lands_a, graph );

				/**
				 * if action do contain p as landmark
				 */

				if( lands_a.isset(p) ){
					//				  std::cout << m_strips_model.actions()[a->index()]->signature() << "NOT first sup of " << m_strips_model.fluents()[p]->signature() << std::endl;

					continue;
				}
				//std::cout << m_strips_model.actions()[a->index()]->signature() << "first sup of " << m_strips_model.fluents()[p]->signature() << std::endl;

				if(k==0)
					lm_set.add( a->prec_set() );
				else
					lm_set.set_intersection( a->prec_set() );


			}

			for (unsigned q : lm_set ) {
				/**
				 * Do not add gn of lands in intial state
				 */
				 /**
				  * chang the m_strips_model.is_in_init to  m_strips_model.is_in_goal
				  */
				if ( !m_collect_lm_in_init_bwd && m_strips_model_bwd.is_in_init(q) ) {
//                if ( !m_collect_lm_in_init && m_strips_model.is_in_goal(q) ) {
					continue;
				}

				if ( m_verbose_bwd )
					std::cout << m_strips_model_bwd.fluents()[q]->signature() << "gn land for " << m_strips_model_bwd.fluents()[p]->signature() << std::endl;
				Landmarks_Graph::Node* nq = graph.node(q);
//				if( nq ){
//					Landmarks_Graph::Node* np = graph.node(p);
//					if( ! np->is_preceded_by(nq) )
//						graph.node(p)->add_precedent_gn( graph.node(q) );
//					if( ! nq->is_required_by(np) )
//						graph.node(q)->add_requiring_gn( graph.node(p) );
				if( nq ){
					Landmarks_Graph::Node* np = graph.node(p);

					if( ! nq->is_preceded_by(np) )
						graph.node(q)->add_precedent_gn( graph.node(p) );
					if( ! np->is_required_by(nq) )
						graph.node(p)->add_requiring_gn( graph.node(q) );
				}
			}

		}


//		if(m_goal_ordering)
//			build_goal_ordering_original( graph );
        if(m_collect_lm_in_init_bwd)
            build_init_ordering_bwd( graph );

	

#ifdef DEBUG
		std::cout << "Landmarks found: " << graph.num_landmarks() << std::endl;
		if ( m_verbose )
			graph.print( std::cout );
#endif
#endif
	}



    void	compute_lm_graph_set_additive_fwd( Landmarks_Graph& graph, State* head_backward=NULL ) {
        std::deque<unsigned> updated;

        Fluent_Vec head_backward_vec;

        if (head_backward!=NULL){
            head_backward_vec=head_backward->fluent_vec();
            graph.remove_landmark();
        } else{
            head_backward_vec=m_strips_model_fwd.goal();
        }

        // 1. Insert goal atoms as landmarks
        for ( Fluent_Vec::const_iterator it = head_backward_vec.begin();
              it != head_backward_vec.end(); it++ ) {
            graph.add_landmark( *it );
            if ( ! m_strips_model_fwd.is_in_init( *it ) ) {
                updated.push_back( *it );
            }
        }


        if( m_only_goals_fwd ){
            if(m_goal_ordering_fwd)
                build_goal_ordering_fwd( graph,head_backward );
            return;
        }

        Bit_Set lm_set( m_strips_model_fwd.num_fluents() );
        Bit_Set processed( m_strips_model_fwd.num_fluents() );

        while ( !updated.empty() ) {
            unsigned p = updated.front();
            updated.pop_front();

            if ( processed.isset(p) ) continue;
            processed.set(p);

            //std::cout << "Processing landmark: " << m_strips_model.fluents()[ p ]->signature() << std::endl;
            const std::vector<const Action*>& add_acts = m_strips_model_fwd.actions_adding( p );
            //std::cout << "Added by " << add_acts.size() << " actions" << std::endl;
            if ( !add_acts.empty() ) {
                lm_set.reset();
                lm_set.add( add_acts[0]->prec_set() );

                for ( unsigned k = 1; k < add_acts.size(); k++ )
                    lm_set.set_intersection( add_acts[k]->prec_set() );
            }

            const std::vector< std::pair< unsigned, const Action*> >& add_acts_ce =
                    m_strips_model_fwd.ceffs_adding( p );

            if ( !add_acts_ce.empty() ) {

                for ( unsigned k = 0; k < add_acts_ce.size(); k++ ) {
                    lm_set.set_intersection( add_acts_ce[k].second->prec_set() );
                    lm_set.set_intersection( add_acts_ce[k].second->ceff_vec()[ add_acts_ce[k].first ]->prec_set() );
                }

            }

            //std::cout << "LM set size: " << lm_set.bits().count_elements() << std::endl;
            for (unsigned q : lm_set ) {
                if ( !m_collect_lm_in_init_fwd && m_strips_model_fwd.is_in_init(q) ) {
                    continue;
                }

                if ( !graph.is_landmark(q) )
                    graph.add_landmark( q );
                //				if ( ! m_strips_model.is_in_goal(p) )
                graph.add_landmark_for( p, q );
                // else{
                // 	graph.add_landmark_for( p, q );
                // 	//graph.node(p)->add_precedent_gn( graph.node(q) );
                // 	//graph.node(q)->add_requiring_gn( graph.node(p) );
                // }
                updated.push_back( q );
            }
        }


        /**
         * Compute Greedy Necessary Orderings
         */
        Bit_Set reach_actions;
        m_reachability_fwd->get_reachable_actions( m_strips_model_fwd.init() , m_strips_model_fwd.goal() , reach_actions  );
        float h_goal;
        State init_s( m_strips_model_fwd );
        init_s.set( m_strips_model_fwd.init() );
        m_h1.eval_fwd( init_s, h_goal );

        for(unsigned p = 1; p <  m_strips_model_fwd.num_fluents(); p++){

            if( ! graph.is_landmark(p) ) continue;
            Action_Ptr_Const_Vec best_supp;

            m_h1.get_best_supporters_fwd( p, best_supp );

            //const std::vector<const Action*>& add_acts = m_strips_model.actions_adding( p );

            lm_set.reset();

            for ( unsigned k = 0; k < best_supp.size(); k++ ){
                const Action* a = best_supp[k];
                /**
                 * if action is reachable
                 */
                if( !reach_actions.isset(a->index()) ) continue;

                Bit_Set lands_a( m_strips_model_fwd.num_fluents() );

                getPCFluentLandmarks_fwd( a->index(), lands_a, graph );

                /**
                 * if action do contain p as landmark
                 */

                if( lands_a.isset(p) ){
                    //				  std::cout << m_strips_model.actions()[a->index()]->signature() << "NOT first sup of " << m_strips_model.fluents()[p]->signature() << std::endl;

                    continue;
                }
                //std::cout << m_strips_model.actions()[a->index()]->signature() << "first sup of " << m_strips_model.fluents()[p]->signature() << std::endl;

                if(k==0)
                    lm_set.add( a->prec_set() );
                else
                    lm_set.set_intersection( a->prec_set() );


            }

            for (unsigned q : lm_set ) {
                /**
                 * Do not add gn of lands in intial state
                 */
                if ( !m_collect_lm_in_init_fwd && m_strips_model_fwd.is_in_init(q) ) {
                    continue;
                }

                if ( m_verbose_fwd )
                    std::cout << m_strips_model_fwd.fluents()[q]->signature() << "gn land for " << m_strips_model_fwd.fluents()[p]->signature() << std::endl;
                Landmarks_Graph::Node* nq = graph.node(q);
                if( nq ){
                    Landmarks_Graph::Node* np = graph.node(p);
                    if( ! np->is_preceded_by(nq) )
                        graph.node(p)->add_precedent_gn( graph.node(q) );
                    if( ! nq->is_required_by(np) )
                        graph.node(q)->add_requiring_gn( graph.node(p) );
                }
            }

        }


        if(m_goal_ordering_fwd)
            build_goal_ordering_fwd( graph );


#ifdef DEBUG
        std::cout << "Landmarks found: " << graph.num_landmarks() << std::endl;
		if ( m_verbose )
			graph.print( std::cout );
#endif

    }
#ifdef DEBUG
    void	compute_lm_graph_set_additive_forward_bwd( Landmarks_Graph& graph ) {

        std::deque<unsigned> updated;

        // 1. Insert goal atoms as landmarks
        for ( Fluent_Vec::const_iterator it = m_strips_model_bwd.goal().begin();
              it != m_strips_model_bwd.goal().end(); it++ ) {
            graph.add_landmark( *it );
            if ( ! m_strips_model_bwd.is_in_init( *it ) ) {
                updated.push_back( *it );
            }
        }


        if( m_only_goals_bwd ){
            if(m_goal_ordering_bwd)
                build_goal_ordering_forward_bwd( graph );
            return;
        }

        Bit_Set lm_set( m_strips_model_bwd.num_fluents() );
        Bit_Set processed( m_strips_model_bwd.num_fluents() );

        while ( !updated.empty() ) {
            unsigned p = updated.front();
            updated.pop_front();

            if ( processed.isset(p) ) continue;
            processed.set(p);

            //std::cout << "Processing landmark: " << m_strips_model.fluents()[ p ]->signature() << std::endl;
            const std::vector<const Action*>& add_acts = m_strips_model_bwd.actions_adding( p );
            //std::cout << "Added by " << add_acts.size() << " actions" << std::endl;
            if ( !add_acts.empty() ) {
                lm_set.reset();
                lm_set.add( add_acts[0]->prec_set() );

                for ( unsigned k = 1; k < add_acts.size(); k++ )
                    lm_set.set_intersection( add_acts[k]->prec_set() );
            }

            const std::vector< std::pair< unsigned, const Action*> >& add_acts_ce =
                    m_strips_model_bwd.ceffs_adding( p );

            if ( !add_acts_ce.empty() ) {

                for ( unsigned k = 0; k < add_acts_ce.size(); k++ ) {
                    lm_set.set_intersection( add_acts_ce[k].second->prec_set() );
                    lm_set.set_intersection( add_acts_ce[k].second->ceff_vec()[ add_acts_ce[k].first ]->prec_set() );
                }

            }

            //std::cout << "LM set size: " << lm_set.bits().count_elements() << std::endl;
            for (unsigned q : lm_set ) {
                if ( !m_collect_lm_in_init_bwd && m_strips_model_bwd.is_in_init(q) ) {
                    continue;
                }

                if ( !graph.is_landmark(q) )
                    graph.add_landmark( q );
                //				if ( ! m_strips_model.is_in_goal(p) )
                graph.add_landmark_for( p, q );
                // else{
                // 	graph.add_landmark_for( p, q );
                // 	//graph.node(p)->add_precedent_gn( graph.node(q) );
                // 	//graph.node(q)->add_requiring_gn( graph.node(p) );
                // }
                updated.push_back( q );
            }
        }


        /**
         * Compute Greedy Necessary Orderings
         */
        Bit_Set reach_actions;
        m_reachability_bwd->get_reachable_actions( m_strips_model_bwd.init() , m_strips_model_bwd.goal() , reach_actions  );
        float h_goal;
        State init_s( m_strips_model_bwd );
        init_s.set( m_strips_model_bwd.init() );
        m_h1.eval_bwd( init_s, h_goal );

        for(unsigned p = 1; p <  m_strips_model_bwd.num_fluents(); p++){

            if( ! graph.is_landmark(p) ) continue;
            Action_Ptr_Const_Vec best_supp;

            m_h1.get_best_supporters_bwd( p, best_supp );

            //const std::vector<const Action*>& add_acts = m_strips_model.actions_adding( p );

            lm_set.reset();

            for ( unsigned k = 0; k < best_supp.size(); k++ ){
                const Action* a = best_supp[k];
                /**
                 * if action is reachable
                 */
                if( !reach_actions.isset(a->index()) ) continue;

                Bit_Set lands_a( m_strips_model_bwd.num_fluents() );

                getPCFluentLandmarks_bwd( a->index(), lands_a, graph );

                /**
                 * if action do contain p as landmark
                 */

                if( lands_a.isset(p) ){
                    //				  std::cout << m_strips_model.actions()[a->index()]->signature() << "NOT first sup of " << m_strips_model.fluents()[p]->signature() << std::endl;

                    continue;
                }
                //std::cout << m_strips_model.actions()[a->index()]->signature() << "first sup of " << m_strips_model.fluents()[p]->signature() << std::endl;

                if(k==0)
                    lm_set.add( a->prec_set() );
                else
                    lm_set.set_intersection( a->prec_set() );


            }

            for (unsigned q : lm_set ) {
                /**
                 * Do not add gn of lands in intial state
                 */
                if ( !m_collect_lm_in_init_bwd && m_strips_model_bwd.is_in_init(q) ) {
                    continue;
                }

                if ( m_verbose_bwd )
                    std::cout << m_strips_model_bwd.fluents()[q]->signature() << "gn land for " << m_strips_model_bwd.fluents()[p]->signature() << std::endl;
                Landmarks_Graph::Node* nq = graph.node(q);
                if( nq ){
                    Landmarks_Graph::Node* np = graph.node(p);
                    if( ! np->is_preceded_by(nq) )
                        graph.node(p)->add_precedent_gn( graph.node(q) );
                    if( ! nq->is_required_by(np) )
                        graph.node(q)->add_requiring_gn( graph.node(p) );
                }
            }

        }


        if(m_goal_ordering_bwd)
            build_goal_ordering_bwd( graph );


#ifdef DEBUG
        std::cout << "Landmarks found: " << graph.num_landmarks() << std::endl;
		if ( m_verbose )
			graph.print( std::cout );
#endif

    }
#endif
#ifdef  DEGUG
    void	compute_lm_graph_set_additive_forward_fwd( Landmarks_Graph& graph ) {

        std::deque<unsigned> updated;

        // 1. Insert goal atoms as landmarks
        for ( Fluent_Vec::const_iterator it = m_strips_model_fwd.goal().begin();
              it != m_strips_model_fwd.goal().end(); it++ ) {
            graph.add_landmark( *it );
            if ( ! m_strips_model_fwd.is_in_init( *it ) ) {
                updated.push_back( *it );
            }
        }


        if( m_only_goals_fwd ){
            if(m_goal_ordering_fwd)
                build_goal_ordering_forward_fwd( graph );
            return;
        }

        Bit_Set lm_set( m_strips_model_fwd.num_fluents() );
        Bit_Set processed( m_strips_model_fwd.num_fluents() );

        while ( !updated.empty() ) {
            unsigned p = updated.front();
            updated.pop_front();

            if ( processed.isset(p) ) continue;
            processed.set(p);

            //std::cout << "Processing landmark: " << m_strips_model.fluents()[ p ]->signature() << std::endl;
            const std::vector<const Action*>& add_acts = m_strips_model_fwd.actions_adding( p );
            //std::cout << "Added by " << add_acts.size() << " actions" << std::endl;
            if ( !add_acts.empty() ) {
                lm_set.reset();
                lm_set.add( add_acts[0]->prec_set() );

                for ( unsigned k = 1; k < add_acts.size(); k++ )
                    lm_set.set_intersection( add_acts[k]->prec_set() );
            }

            const std::vector< std::pair< unsigned, const Action*> >& add_acts_ce =
                    m_strips_model_fwd.ceffs_adding( p );

            if ( !add_acts_ce.empty() ) {

                for ( unsigned k = 0; k < add_acts_ce.size(); k++ ) {
                    lm_set.set_intersection( add_acts_ce[k].second->prec_set() );
                    lm_set.set_intersection( add_acts_ce[k].second->ceff_vec()[ add_acts_ce[k].first ]->prec_set() );
                }

            }

            //std::cout << "LM set size: " << lm_set.bits().count_elements() << std::endl;
            for (unsigned q : lm_set ) {
                if ( !m_collect_lm_in_init_fwd&& m_strips_model_fwd.is_in_init(q) ) {
                    continue;
                }

                if ( !graph.is_landmark(q) )
                    graph.add_landmark( q );
                //				if ( ! m_strips_model.is_in_goal(p) )
                graph.add_landmark_for( p, q );
                // else{
                // 	graph.add_landmark_for( p, q );
                // 	//graph.node(p)->add_precedent_gn( graph.node(q) );
                // 	//graph.node(q)->add_requiring_gn( graph.node(p) );
                // }
                updated.push_back( q );
            }
        }


        /**
         * Compute Greedy Necessary Orderings
         */
        Bit_Set reach_actions;
        m_reachability_fwd->get_reachable_actions( m_strips_model_fwd.init() , m_strips_model_fwd.goal() , reach_actions  );
        float h_goal;
        State init_s( m_strips_model_fwd );
        init_s.set( m_strips_model_fwd.init() );
        m_h1.eval_fwd( init_s, h_goal );

        for(unsigned p = 1; p <  m_strips_model_fwd.num_fluents(); p++){

            if( ! graph.is_landmark(p) ) continue;
            Action_Ptr_Const_Vec best_supp;

            m_h1.get_best_supporters_fwd( p, best_supp );

            //const std::vector<const Action*>& add_acts = m_strips_model.actions_adding( p );

            lm_set.reset();

            for ( unsigned k = 0; k < best_supp.size(); k++ ){
                const Action* a = best_supp[k];
                /**
                 * if action is reachable
                 */
                if( !reach_actions.isset(a->index()) ) continue;

                Bit_Set lands_a( m_strips_model_fwd.num_fluents() );

                getPCFluentLandmarks_fwd( a->index(), lands_a, graph );

                /**
                 * if action do contain p as landmark
                 */

                if( lands_a.isset(p) ){
                    //				  std::cout << m_strips_model.actions()[a->index()]->signature() << "NOT first sup of " << m_strips_model.fluents()[p]->signature() << std::endl;

                    continue;
                }
                //std::cout << m_strips_model.actions()[a->index()]->signature() << "first sup of " << m_strips_model.fluents()[p]->signature() << std::endl;

                if(k==0)
                    lm_set.add( a->prec_set() );
                else
                    lm_set.set_intersection( a->prec_set() );


            }

            for (unsigned q : lm_set ) {
                /**
                 * Do not add gn of lands in intial state
                 */
                if ( !m_collect_lm_in_init_fwd && m_strips_model_fwd.is_in_init(q) ) {
                    continue;
                }

                if ( m_verbose_fwd )
                    std::cout << m_strips_model_fwd.fluents()[q]->signature() << "gn land for " << m_strips_model_fwd.fluents()[p]->signature() << std::endl;
                Landmarks_Graph::Node* nq = graph.node(q);
                if( nq ){
                    Landmarks_Graph::Node* np = graph.node(p);
                    if( ! np->is_preceded_by(nq) )
                        graph.node(p)->add_precedent_gn( graph.node(q) );
                    if( ! nq->is_required_by(np) )
                        graph.node(q)->add_requiring_gn( graph.node(p) );
                }
            }

        }


        if(m_goal_ordering_fwd)
            build_goal_ordering_fwd( graph );


#ifdef DEBUG
        std::cout << "Landmarks found: " << graph.num_landmarks() << std::endl;
		if ( m_verbose )
			graph.print( std::cout );
#endif

    }
#endif
protected:

	void getFluentLandmarks( unsigned p, Bit_Set& landmarks, Landmarks_Graph& graph ){
		
		if( ! graph.is_landmark( p ) ) return;

		for(std::vector<Landmarks_Graph::Node*>::const_iterator it = graph.node( p )->preceded_by().begin(); 
		    it != graph.node( p )->preceded_by().end(); it++ ){
			unsigned fl = (*it)->fluent();

			if( landmarks.isset( fl ) ) continue;
			//std::cout << m_strips_model.fluents()[fl]->signature() << " " <<std::endl; 
			landmarks.set( fl );
			getFluentLandmarks( fl, landmarks, graph);
		}
		
	}
	
	void getPCFluentLandmarks_bwd(unsigned act_idx,
				  Bit_Set &landmarks, Landmarks_Graph& graph ) {

	  // std::cout << "Action " << m_strips_model.actions()[act_idx]->signature() << " Land: ";
		const Fluent_Vec& prec = m_strips_model_bwd.actions()[act_idx]->prec_vec();
                for(unsigned i = 0; i < prec.size(); i++) {
                        landmarks.set( prec[i] );
			//	std::cout << m_strips_model.fluents()[prec[i]]->signature() << " " <<std::endl; 
			getFluentLandmarks( prec[i], landmarks, graph);
                }
		//	std::cout << std::endl;


        }
    void getPCFluentLandmarks_fwd(unsigned act_idx,
                              Bit_Set &landmarks, Landmarks_Graph& graph ) {

        // std::cout << "Action " << m_strips_model.actions()[act_idx]->signature() << " Land: ";
        const Fluent_Vec& prec = m_strips_model_fwd.actions()[act_idx]->prec_vec();
        for(unsigned i = 0; i < prec.size(); i++) {
            landmarks.set( prec[i] );
            //	std::cout << m_strips_model.fluents()[prec[i]]->signature() << " " <<std::endl;
            getFluentLandmarks( prec[i], landmarks, graph);
        }
        //	std::cout << std::endl;


    }


protected:

    const bwd_Search_Problem&           search_prob_bwd;
	const Fwd_Search_Problem&           search_prob_fwd;


	//const STRIPS_Problem&			m_strips_model;
    const STRIPS_Problem&			m_strips_model_bwd;
    const STRIPS_Problem&			m_strips_model_fwd;

  //  bool                                    m_only_goals;
    bool                                    m_only_goals_bwd;
    bool                                    m_only_goals_fwd;


	//bool                                    m_goal_ordering;
    bool                                    m_goal_ordering_bwd;
    bool                                    m_goal_ordering_fwd;


	//Reachability_Test*                      m_reachability;
    Reachability_Test*                      m_reachability_bwd;
    Reachability_Test*                      m_reachability_fwd;


    H_Max                                   m_h1;
//    H_Max                                   m_h1_bwd;
//    H_Max                                   m_h1_fwd;


	//bool					m_verbose;
    bool					m_verbose_bwd;
    bool					m_verbose_fwd;

	//bool					m_collect_lm_in_init;
    bool					m_collect_lm_in_init_bwd;
    bool					m_collect_lm_in_init_fwd;

};

}

}

#endif // landmark_graph_generator.hxx
