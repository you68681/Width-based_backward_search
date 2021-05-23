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

#ifndef __LANDMARK_COUNT__
#define __LANDMARK_COUNT__

#include <aptk/search_prob.hxx>
#include <aptk/heuristic.hxx>
#include <landmark_graph.hxx>
#include <landmark_graph_manager.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <action.hxx>

#include <stack>
#include <vector>
#include <deque>
#include <iosfwd>

namespace aptk {

    namespace agnostic {

        template <typename bwd_Search_Problem, typename Fwd_Search_Problem>
        class Landmarks_Count_Heuristic : public Heuristic<State> {
        public:
            typedef         aptk::agnostic::Landmarks_Graph_Manager<bwd_Search_Problem,Fwd_Search_Problem>   Landmarks_Graph_Manager_bwd;
            typedef         aptk::agnostic::Landmarks_Graph_Manager<bwd_Search_Problem,Fwd_Search_Problem>   Landmarks_Graph_Manager_fwd;

//            Landmarks_Count_Heuristic( const bwd_Search_Problem& prob )
//                    : Heuristic<State>( prob ), m_strips_model_bwd( prob.task() ), m_max_value_bwd(0)
//            {
//                m_graph_bwd = NULL;
//                m_lgm_bwd = NULL;
//                m_in_leafs_bwd.resize( m_strips_model_bwd.num_fluents() );
//            }
//            Landmarks_Count_Heuristic( const Fwd_Search_Problem& prob )
//                    : Heuristic<State>( prob ), m_strips_model_fwd( prob.task() ), m_max_value_fwd(0)
//            {
//                m_graph_fwd = NULL;
//                m_lgm_fwd = NULL;
//                is_bwd=false;
//                m_in_leafs_fwd.resize( m_strips_model_fwd.num_fluents() );
//            }

            Landmarks_Count_Heuristic(const bwd_Search_Problem& prob_bwd, const Fwd_Search_Problem& prob_fwd)
                    : Heuristic<State>( prob_bwd,prob_fwd ),m_strips_model_bwd(prob_bwd.task()),m_strips_model_fwd(prob_fwd.task()),m_max_value_bwd(0),m_max_value_fwd(0)
            {
                m_graph_bwd = NULL;
                m_graph_fwd = NULL;

                m_lgm_bwd = NULL;
                m_lgm_fwd = NULL;

            //    is_bwd=false;
                m_in_leafs_bwd.resize( m_strips_model_bwd.num_fluents() );
                m_in_leafs_fwd.resize( m_strips_model_fwd.num_fluents() );
            }




//            Landmarks_Count_Heuristic( const bwd_Search_Problem& prob, Landmarks_Graph* lg )
//                    : Heuristic<State>( prob ), m_strips_model_bwd( prob.task() )
//            {
//                m_graph_bwd = lg;
//                m_in_leafs_bwd.resize( m_strips_model_bwd.num_fluents() );
//            }
//
//            Landmarks_Count_Heuristic( const Fwd_Search_Problem& prob, Landmarks_Graph* lg )
//                    : Heuristic<State>( prob ), m_strips_model_fwd( prob.task() )
//            {
//                m_graph_fwd = lg;
//                is_bwd = false;
//                m_in_leafs_fwd.resize( m_strips_model_fwd.num_fluents() );
//            }

            Landmarks_Count_Heuristic( const bwd_Search_Problem& prob_bwd,  Landmarks_Graph* lg_bwd ,const Fwd_Search_Problem& prob_fwd,  Landmarks_Graph* lg_fwd   )
                    :Heuristic<State>( prob_bwd,prob_fwd ), m_strips_model_bwd( prob_bwd.task() ),m_strips_model_fwd( prob_fwd.task() )
            {
                m_graph_bwd = lg_bwd;
                m_graph_fwd = lg_fwd;


                m_in_leafs_bwd.resize( m_strips_model_bwd.num_fluents() );
                m_in_leafs_fwd.resize( m_strips_model_fwd.num_fluents() );
            }




            virtual ~Landmarks_Count_Heuristic() {

            }


            void set_graph_bwd( Landmarks_Graph* lg ){  m_graph_bwd = lg ; }
            void set_graph_fwd( Landmarks_Graph* lg ){  m_graph_fwd = lg ; }

            /**
             * If manager is defined, landmark graph is recomputed at every node.
             * If it's undefined, a land manager has to be used externally by the search algorithm to update
             * the state of the graph before counting
             */
            void set_graph_manager_bwd( Landmarks_Graph_Manager_bwd * lg ){
                m_lgm_bwd = lg;
                set_graph_bwd( m_lgm_bwd->graph() );
            }

            void set_graph_manager_fwd( Landmarks_Graph_Manager_fwd * lg ){
                m_lgm_fwd = lg;
                set_graph_fwd( m_lgm_fwd->graph() );
            }

            template <typename Search_Node>
            void update_graph_bwd( const Search_Node* n ){

                static std::stack<const Search_Node*> path;

                //NIR: recover path
                const Search_Node* tmp = n;
                while( tmp ){
                    path.push( tmp );
                    tmp = tmp->m_parent;
                }

                //NIR: reset graph
                m_lgm_bwd->reset_graph_bwd() ;
             //   m_lgm->reset_graph();

                while(!path.empty()){
                    tmp = path.top();
                    path.pop();

                    //NIR: In the root state initialize the graph with the state information, otherwise use action
                    if( ! tmp->m_parent )
                        m_lgm_bwd->apply_state_bwd( tmp->state().fluent_vec() ) ;
                    else
                        m_lgm_bwd->apply_action_bwd( &(tmp->state()), tmp->action() );
                }

            }
            template <typename Search_Node>
            void update_graph_fwd( const Search_Node* n ){

                static std::stack<const Search_Node*> path;

                //NIR: recover path
                const Search_Node* tmp = n;
                while( tmp ){
                    path.push( tmp );
                    tmp = tmp->m_parent;
                }

                //NIR: reset graph
                 m_lgm_fwd->reset_graph_bwd();
                //   m_lgm->reset_graph();

                while(!path.empty()){
                    tmp = path.top();
                    path.pop();

                    //NIR: In the root state initialize the graph with the state information, otherwise use action
                    if( ! tmp->m_parent )
                         m_lgm_fwd->apply_state_fwd( tmp->state().fluent_vec() );
                    else
                         m_lgm_fwd->apply_action_fwd( &(tmp->state()), tmp->action() );
                }

            }


            template <typename Search_Node>
            void eval( const Search_Node* n, float& h_val, std::vector<Action_Idx>& pref_ops) {
                update_graph(n);
                unsigned h;
                eval(n->state(), h, pref_ops);
                h_val = h;

            }


            template <typename Search_Node>
            void eval( const Search_Node* n, float& h_val ) {

                update_graph(n);
                unsigned h;
                eval(n->state(),h);
                h_val = h;
            }

            void eval( const State& s, float& h_val ) {
                unsigned h;
                eval(s,h);
                h_val = h;
            }
            /** chao edit
             *
             * @param s
             * @param h_val
             * @param parent
             */

            virtual void eval_bwd( const State& s, unsigned& h_val,  Fluent_Vec parent) {
                Landmarks_Graph*			m_graph=m_graph_bwd ;


                if ( !m_graph ) return;
                h_val = 0;
                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin(); it != m_graph->nodes().end(); it++ ) {
                    Landmarks_Graph::Node*n = *it;
                    /** chao edit
                     *
                     */
//			if (m_strips_model.is_in_goal(n->fluent()))
//                continue;
                    if( ! n->is_consumed() ) {
                        h_val++;
                        //std::cout<<h_val<<m_strips_model.fluents()[n->fluent()]->signature()<<std::endl;
                    }

                    if( !n->required_by_gn().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ ) {
                            //Landmarks_Graph::Node *n_r = *it_r;
                            if( ! (*it_r)->is_consumed_once() ){
                                //if (!n_r->is_consumed_once()) {
                                //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                                h_val++;
                                //std::cout<<h_val<<m_strips_model.fluents()[n_r->fluent()]->signature()<<std::endl;
                            }
                        }
                    }


                    if( ! n->required_by().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ ) {
                            //Landmarks_Graph::Node *n_l = *it_r;
                            if( ! (*it_r)->is_consumed() ){
                                //if (!n_l->is_consumed_once()) {
                                //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                                h_val++;
                                //	break;
                                //std::cout<<h_val<<m_strips_model.fluents()[n_l->fluent()]->signature()<<std::endl;
                            }
                        }
                    }





                }

                for (unsigned p: parent){
                    if ( !this->m_strips_model_bwd.is_in_init(p)  ){            //!this->m_strips_model.is_in_init(p)
                        h_val++;
                    }
                }

            }



            virtual void eval_fwd( const State& s, unsigned& h_val,  Fluent_Vec parent) {
                Landmarks_Graph*			m_graph=m_graph_fwd;


                if ( !m_graph ) return;
                h_val = 0;
                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin(); it != m_graph->nodes().end(); it++ ) {
                    Landmarks_Graph::Node*n = *it;
                    /** chao edit
                     *
                     */
//			if (m_strips_model.is_in_goal(n->fluent()))
//                continue;
                    if( ! n->is_consumed() ) {
                        h_val++;
                        //std::cout<<h_val<<m_strips_model.fluents()[n->fluent()]->signature()<<std::endl;
                    }

                    if( !n->required_by_gn().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ ) {
                            //Landmarks_Graph::Node *n_r = *it_r;
                            if( ! (*it_r)->is_consumed_once() ){
                                //if (!n_r->is_consumed_once()) {
                                //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                                h_val++;
                                //std::cout<<h_val<<m_strips_model.fluents()[n_r->fluent()]->signature()<<std::endl;
                            }
                        }
                    }


                    if( ! n->required_by().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ ) {
                            //Landmarks_Graph::Node *n_l = *it_r;
                            if( ! (*it_r)->is_consumed() ){
                                //if (!n_l->is_consumed_once()) {
                                //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                                h_val++;
                                //	break;
                                //std::cout<<h_val<<m_strips_model.fluents()[n_l->fluent()]->signature()<<std::endl;
                            }
                        }
                    }





                }

                for (unsigned p: parent){
                    if (  !this->m_strips_model_fwd.is_in_init(p)  ){            //!this->m_strips_model.is_in_init(p)
                        h_val++;
                    }
                }

            }
//	/** chao edit
//	 *
//	 */
//    virtual void eval_count( const State* s, unsigned& h_val ) {
//        h_val = 0;
//        for (unsigned p : s->fluent_vec()){
//            if (!problem().init_state()->entails(p)){
//                h_val++;
//            }
//        }
//    }
            /** chao edit test
             *
             * @param s
             * @param h_val
             */

            virtual void eval_fwd( const State& s, unsigned& h_val ) {
                if (!m_graph_fwd) return;

                h_val = 0;
                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph_fwd->nodes().begin(); it != m_graph_fwd->nodes().end(); it++ ) {
                    Landmarks_Graph::Node*n = *it;
                    if( ! n->is_consumed()  ) {
                        h_val++;

                    }

                    if( !n->required_by_gn().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ ){
                            if( ! (*it_r)->is_consumed_once()  ){
                                //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                                h_val++;
                            }
                        }
                    }


                    if( ! n->required_by().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ ){
                            if( ! (*it_r)->is_consumed()   ){
                                //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                                h_val++;
                                //	break;
                            }
                        }
                    }



                }
            }

            virtual void eval_edit_bwd( const State& s ) {
                unsigned h_val=0;
                Landmarks_Graph*			m_graph= m_graph_bwd ;


                if (!m_graph) return;

                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin(); it != m_graph->nodes().end(); it++ ) {
                    Landmarks_Graph::Node*n = *it;
                    if( ! n->is_consumed() ) {
                        h_val++;

                        //std::cout<<h_val<<m_strips_model.fluents()[n->fluent()]->signature()<<std::endl;

                        std::cout<<h_val<< m_strips_model_bwd.fluents()[n->fluent()]->signature() <<std::endl;
                    }

                    if( !n->required_by_gn().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ ) {
                            Landmarks_Graph::Node *n_r = *it_r;
                            //if( ! (*it_r)->is_consumed_once() ){
                            if (!n_r->is_consumed_once()) {
                                h_val++;

                               // std::cout<<h_val<<m_strips_model.fluents()[n_r->fluent()]->signature()<<std::endl;
                                std::cout<<h_val<< m_strips_model_bwd.fluents()[n_r->fluent()]->signature() <<std::endl;
                            }
                        }
                    }


                    if( ! n->required_by().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ ) {
                            Landmarks_Graph::Node *n_l = *it_r;
                            //if( ! (*it_r)->is_consumed() ){
                            if (!n_l->is_consumed_once()) {
                                //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                                h_val++;
                                //	break;
                                //std::cout<<h_val<<m_strips_model.fluents()[n_l->fluent()]->signature()<<std::endl;
                                std::cout<<h_val<< m_strips_model_bwd.fluents()[n_l->fluent()]->signature() <<std::endl;
                            }
                        }
                    }



                }
            }

            virtual void eval_edit_fwd( const State& s ) {
                unsigned h_val=0;
                Landmarks_Graph*			m_graph=m_graph_fwd;


                if (!m_graph) return;

                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin(); it != m_graph->nodes().end(); it++ ) {
                    Landmarks_Graph::Node*n = *it;
                    if( ! n->is_consumed() ) {
                        h_val++;

                        //std::cout<<h_val<<m_strips_model.fluents()[n->fluent()]->signature()<<std::endl;

                        std::cout<<h_val<< m_strips_model_fwd.fluents()[n->fluent()]->signature() <<std::endl;
                    }

                    if( !n->required_by_gn().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ ) {
                            Landmarks_Graph::Node *n_r = *it_r;
                            //if( ! (*it_r)->is_consumed_once() ){
                            if (!n_r->is_consumed_once()) {
                                h_val++;

                                // std::cout<<h_val<<m_strips_model.fluents()[n_r->fluent()]->signature()<<std::endl;
                                std::cout<<h_val<< m_strips_model_fwd.fluents()[n_r->fluent()]->signature() <<std::endl;
                            }
                        }
                    }


                    if( ! n->required_by().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ ) {
                            Landmarks_Graph::Node *n_l = *it_r;
                            //if( ! (*it_r)->is_consumed() ){
                            if (!n_l->is_consumed_once()) {
                                //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                                h_val++;
                                //	break;
                                //std::cout<<h_val<<m_strips_model.fluents()[n_l->fluent()]->signature()<<std::endl;
                                std::cout<<h_val<< m_strips_model_fwd.fluents()[n_l->fluent()]->signature() <<std::endl;
                            }
                        }
                    }



                }
            }


/** chao add goal count
 *
 * @param s
 * @param h_val
 * @param pref_ops
 */
            virtual Fluent_Vec eval_goal_count_bwd( const State& s, Fluent_Vec unachived_goal ) {
                Landmarks_Graph*			m_graph=( m_graph_bwd );

                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin(); it != m_graph->nodes().end(); it++ ) {
                    Landmarks_Graph::Node*n = *it;
                    if( ! n->is_consumed() ) {
                        unachived_goal.push_back(n->fluent());
                    }

                    if( !n->required_by_gn().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ ) {
                            Landmarks_Graph::Node *n_r = *it_r;
                            //if( ! (*it_r)->is_consumed_once() ){
                            if (!n_r->is_consumed_once()) {

                                unachived_goal.push_back(n->fluent());
                            }
                        }
                    }


                    if( ! n->required_by().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ ) {
                            Landmarks_Graph::Node *n_l = *it_r;
                            //if( ! (*it_r)->is_consumed() ){
                            if (!n_l->is_consumed_once()) {
                                unachived_goal.push_back(n->fluent());
                            }
                        }
                    }



                }
                return unachived_goal;
            }

            virtual Fluent_Vec eval_goal_count_fwd( const State& s, Fluent_Vec unachived_goal ) {
                Landmarks_Graph*			m_graph=( m_graph_fwd );

                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin(); it != m_graph->nodes().end(); it++ ) {
                    Landmarks_Graph::Node*n = *it;
                    if( ! n->is_consumed() ) {
                        unachived_goal.push_back(n->fluent());
                    }

                    if( !n->required_by_gn().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ ) {
                            Landmarks_Graph::Node *n_r = *it_r;
                            //if( ! (*it_r)->is_consumed_once() ){
                            if (!n_r->is_consumed_once()) {

                                unachived_goal.push_back(n->fluent());
                            }
                        }
                    }


                    if( ! n->required_by().empty() ){
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ ) {
                            Landmarks_Graph::Node *n_l = *it_r;
                            //if( ! (*it_r)->is_consumed() ){
                            if (!n_l->is_consumed_once()) {
                                unachived_goal.push_back(n->fluent());
                            }
                        }
                    }



                }
                return unachived_goal;
            }

            void eval( const State& s, float& h_val,  std::vector<Action_Idx>& pref_ops ) {
                unsigned h;
                eval(s,h,pref_ops);
                h_val = h;
            }

            /**
             * Graph should be updated already, otherwise use eval(Node,...)
             */
            virtual void eval_bwd( const State& s, unsigned& h_val,  std::vector<Action_Idx>& pref_ops ) {
                Landmarks_Graph*			m_graph=m_graph_bwd ;

                if (!m_graph) return;
                m_in_leafs_bwd.reset() ;
          //      m_in_leafs.reset();

                h_val = 0;
                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin(); it != m_graph->nodes().end(); it++ ) {

                    Landmarks_Graph::Node*n = *it;
                    if( ! n->is_consumed() ) {
                        if( n->required_by().empty() ){
                            //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                            h_val++;
                        }
                        else
                            //for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ )
                            //if( ! (*it_r)->is_consumed() )
                            h_val++;

                        if(  n->are_precedences_consumed() )
                           m_in_leafs_bwd.set( n->fluent() ) ;
                         //   m_in_leafs.set( n->fluent() );

                    }
                }

                std::vector< aptk::Action_Idx >	app_set;

                this->problem_bwd().applicable_set_v2( s, app_set );

                for ( unsigned i = 0; i < app_set.size(); i++ ) {
                    int a = app_set[i];

                    const Action& act = *(   m_strips_model_bwd.actions()[a]  );
                    for ( Fluent_Vec::const_iterator it2 = act.add_vec().begin();
                          it2 != act.add_vec().end(); it2++ )
                        if (  m_in_leafs_bwd.isset( *it2 ) ) {           //m_in_leafs.isset( *it2 )
                             pref_ops.push_back( act.index() );
                            //m_in_leafs.unset(*it2); // Just one supporter for leaf landmark
                            break;
                        }

                }
            }
            virtual void eval_fwd( const State& s, unsigned& h_val,  std::vector<Action_Idx>& pref_ops ) {
                Landmarks_Graph*			m_graph= m_graph_fwd;

                if (!m_graph) return;
                 m_in_leafs_fwd.reset();
                //      m_in_leafs.reset();

                h_val = 0;
                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin(); it != m_graph->nodes().end(); it++ ) {

                    Landmarks_Graph::Node*n = *it;
                    if( ! n->is_consumed() ) {
                        if( n->required_by().empty() ){
                            //std::cout << " "<< m_strips_model.fluents()[ n->fluent() ]->signature() << std::endl;
                            h_val++;
                        }
                        else
                            //for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ )
                            //if( ! (*it_r)->is_consumed() )
                            h_val++;

                        if(  n->are_precedences_consumed() )
                            m_in_leafs_fwd.set( n->fluent() );
                        //   m_in_leafs.set( n->fluent() );

                    }
                }

                std::vector< aptk::Action_Idx >	app_set;
                this->problem_fwd().applicable_set_v2( s, app_set );

                for ( unsigned i = 0; i < app_set.size(); i++ ) {
                    int a = app_set[i];

                    const Action& act = *(   m_strips_model_fwd.actions()[a] )   ;
                    for ( Fluent_Vec::const_iterator it2 = act.add_vec().begin();
                          it2 != act.add_vec().end(); it2++ )
                        if ( m_in_leafs_fwd.isset( *it2 ) ) {           //m_in_leafs.isset( *it2 )
                            pref_ops.push_back( act.index() );
                            //m_in_leafs.unset(*it2); // Just one supporter for leaf landmark
                            break;
                        }

                }
            }

            unsigned max_value_bwd() const { return  m_max_value_bwd ; }   //m_max_value
            unsigned max_value_fwd() const { return  m_max_value_fwd; }   //m_max_value
        protected:

            void update_max_value_bwd(){

                unsigned m_max_value=0;
                Landmarks_Graph*			m_graph=m_graph_bwd;


                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin();
                      it != m_graph->nodes().end(); it++ ) {

                    Landmarks_Graph::Node*n = *it;
                    m_max_value++;


                    if( !n->required_by_gn().empty() )
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ )
                            m_max_value++;



                    if( ! n->required_by().empty() )
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ )
                            m_max_value++;


                }
                m_max_value_bwd = m_max_value;
            }
            void update_max_value_fwd(){

                unsigned m_max_value=0;
                Landmarks_Graph*			m_graph= m_graph_fwd;


                for ( std::vector< Landmarks_Graph::Node* >::const_iterator it = m_graph->nodes().begin();
                      it != m_graph->nodes().end(); it++ ) {

                    Landmarks_Graph::Node*n = *it;
                    m_max_value++;


                    if( !n->required_by_gn().empty() )
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by_gn().begin(); it_r != n->required_by_gn().end(); it_r++ )
                            m_max_value++;



                    if( ! n->required_by().empty() )
                        for( std::vector< Landmarks_Graph::Node* >::const_iterator it_r = n->required_by().begin(); it_r != n->required_by().end(); it_r++ )
                            m_max_value++;


                }
                 m_max_value_fwd = m_max_value;
            }

        protected:

          //  const STRIPS_Problem&			m_strips_model;
            const STRIPS_Problem&			m_strips_model_bwd;
            const STRIPS_Problem&			m_strips_model_fwd;
            //Landmarks_Graph*			m_graph;
            Landmarks_Graph*			m_graph_bwd;
            Landmarks_Graph*			m_graph_fwd;
           // Landmarks_Graph_Manager*                m_lgm;
            Landmarks_Graph_Manager_bwd *                m_lgm_bwd;
            Landmarks_Graph_Manager_fwd*                m_lgm_fwd;
          //  Bit_Set					m_in_leafs;
            Bit_Set					m_in_leafs_bwd;
            Bit_Set					m_in_leafs_fwd;
            unsigned                                m_max_value_bwd;
            unsigned                                m_max_value_fwd;

          //  bool                   is_bwd=true;
        };

    }

}


#endif // landmark_count.hxx
