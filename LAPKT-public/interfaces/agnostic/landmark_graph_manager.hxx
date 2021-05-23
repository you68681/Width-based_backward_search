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

#ifndef __LANDMARK_GRAPH_MANAGER__
#define __LANDMARK_GRAPH_MANAGER__

#include <aptk/search_prob.hxx>
#include <aptk/heuristic.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <landmark_graph.hxx>
#include <action.hxx>
#include <vector>
#include <deque>
#include <iosfwd>

namespace aptk {

namespace agnostic {



template <typename bwd_Search_Model, typename Fwd_Search_Model >
class Landmarks_Graph_Manager  {
public:

	Landmarks_Graph_Manager( const bwd_Search_Model& prob_bwd, Landmarks_Graph* lg_bwd,const Fwd_Search_Model& prob_fwd,Landmarks_Graph* lg_fwd )
	:  m_strips_model_bwd( prob_bwd.task() ), m_strips_model_fwd( prob_fwd.task() ),search_prob_bwd(prob_bwd),search_prob_fwd(prob_fwd)
	{
		m_graph_bwd = lg_bwd;
        m_graph_fwd = lg_fwd;
	}

	virtual ~Landmarks_Graph_Manager() {
	}

	
	void reset_graph_bwd(){
		m_graph_bwd->unconsume_all();
	}
    void reset_graph_fwd(){
        m_graph_fwd->unconsume_all();
    }

	void update_graph( Bool_Vec_Ptr*& keep_consumed, Bool_Vec_Ptr*& keep_unconsumed ){
		if(keep_consumed)
			for( Bool_Vec_Ptr::iterator it = keep_consumed->begin(); it != keep_consumed->end(); it++)
				**it = true;
		if(keep_unconsumed)
			for( Bool_Vec_Ptr::iterator it = keep_unconsumed->begin(); it != keep_unconsumed->end(); it++)
				**it = false;
	}

	void undo_graph( Bool_Vec_Ptr*& undo_consumed, Bool_Vec_Ptr*& undo_unconsumed ){
		if(undo_consumed)
			for( Bool_Vec_Ptr::iterator it = undo_consumed->begin(); it != undo_consumed->end(); it++)
				**it = false;
		if(undo_unconsumed)
			for( Bool_Vec_Ptr::iterator it = undo_unconsumed->begin(); it != undo_unconsumed->end(); it++)
				**it = true;
	}

	void apply_action_bwd( State* s, Action_Idx a_idx, Bool_Vec_Ptr*& keep_consumed, Bool_Vec_Ptr*& keep_unconsumed ){
		const Action* a = m_strips_model_bwd.actions()[ a_idx ];
#ifdef DEBUG

		/**original version
		 *
		const Fluent_Vec& add = a->add_vec();
		const Fluent_Vec& del = a->del_vec();

		for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
			unsigned p = *it_add;
			
			if( m_graph->is_landmark(p) ){
				Landmarks_Graph::Node* n = m_graph->node(p);
				if( !n->is_consumed() )
					if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
						if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
						//std::cout << "\t -- "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;														       
						n->consume( );
						keep_consumed->push_back( n->is_consumed_ptr() );
					}
			}
		}

		for(Fluent_Vec::const_iterator it_del = del.begin(); it_del != del.end(); it_del++){
			unsigned p = *it_del;
						
			if( m_graph->is_landmark(p) ){
				bool unconsume = false;
				Landmarks_Graph::Node* n = m_graph->node(p);

				if( n->is_consumed() )
					if(  m_strips_model.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
						unconsume = true;
				

				if(unconsume){
					//std::cout << "\t ++ "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;					
					if( !keep_unconsumed  ) keep_unconsumed = new Bool_Vec_Ptr;

					n->unconsume( );
					keep_unconsumed->push_back( n->is_consumed_ptr() );
					

				
				}
			}
		}
        */
#endif
		/** chao edit
		 *
		 */
        const Fluent_Vec& add = a->add_vec();
        const Fluent_Vec& pre = a->prec_vec();
        bool fix_point_flag= true;
        while (fix_point_flag){
            bool tem_flag      = false;
            for(Fluent_Vec::const_iterator it_pre = pre.begin(); it_pre != pre.end(); it_pre++){
                unsigned p = *it_pre;

                if( m_graph_bwd->is_landmark(p) ){
                    Landmarks_Graph::Node* n = m_graph_bwd->node(p);
                    if( !n->is_consumed() )
                        if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
                            if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
                            //std::cout << "\t -- "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
                            n->consume( );
                            keep_consumed->push_back( n->is_consumed_ptr() );

                            fix_point_flag= true;
                            tem_flag      =true;
                            continue;
                        }
                }
                if (!tem_flag)
                    fix_point_flag= false;
            }
        }
#if 0
//        for(Fluent_Vec::const_iterator it_pre = pre.begin(); it_pre != pre.end(); it_pre++){
//            unsigned p = *it_pre;
//
//            if( m_graph->is_landmark(p) ){
//                Landmarks_Graph::Node* n = m_graph->node(p);
//                if( !n->is_consumed() )
//                    if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
//                        if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
//                        //std::cout << "\t -- "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
//                        n->consume( );
//                        keep_consumed->push_back( n->is_consumed_ptr() );
//                    }
//            }
//        }
#endif
        for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
            unsigned p = *it_add;

            if( m_graph_bwd->is_landmark(p) ){
                bool unconsume = false;
                Landmarks_Graph::Node* n = m_graph_bwd->node(p);

                if( n->is_consumed() )
                    /**chao edit change the m_strips_model.is_in_goal to m_strips_model.is_in_init
                     *
                     */
                    //if(  m_strips_model.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                     if(  m_strips_model_bwd.is_in_init(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                        unconsume = true;


                if(unconsume){
                    //std::cout << "\t ++ "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
                    if( !keep_unconsumed  ) keep_unconsumed = new Bool_Vec_Ptr;

                    n->unconsume( );
                    keep_unconsumed->push_back( n->is_consumed_ptr() );



                }
            }
        }

        if( !a->ceff_vec().empty() ){
			for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
				Conditional_Effect* ce = a->ceff_vec()[i];
				if( ce->can_be_applied_on( *s ) )
					apply_cond_eff_bwd(s,ce, keep_consumed, keep_unconsumed);
			}
		}

	}
    void apply_action_fwd( State* s, Action_Idx a_idx, Bool_Vec_Ptr*& keep_consumed, Bool_Vec_Ptr*& keep_unconsumed ){
        const Action* a = m_strips_model_fwd.actions()[ a_idx ];


        const Fluent_Vec& add = a->add_vec();
        const Fluent_Vec& del = a->del_vec();

        for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
            unsigned p = *it_add;

            if( m_graph_fwd->is_landmark(p) ){
                Landmarks_Graph::Node* n = m_graph_fwd->node(p);
                if( !n->is_consumed() )
                    if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
                        if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
                        //std::cout << "\t -- "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
                        n->consume( );
                        keep_consumed->push_back( n->is_consumed_ptr() );
                    }
            }
        }

        for(Fluent_Vec::const_iterator it_del = del.begin(); it_del != del.end(); it_del++){
            unsigned p = *it_del;

            if( m_graph_fwd->is_landmark(p) ){
                bool unconsume = false;
                Landmarks_Graph::Node* n = m_graph_fwd->node(p);

                if( n->is_consumed() )
                    if(  m_strips_model_fwd.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                        unconsume = true;


                if(unconsume){
                    //std::cout << "\t ++ "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
                    if( !keep_unconsumed  ) keep_unconsumed = new Bool_Vec_Ptr;

                    n->unconsume( );
                    keep_unconsumed->push_back( n->is_consumed_ptr() );



                }
            }
        }
#ifdef DEBUG
        /** chao edit
         *
         */
//        const Fluent_Vec& add = a->add_vec();
//        const Fluent_Vec& pre = a->prec_vec();
//        bool fix_point_flag= true;
//        while (fix_point_flag){
//            bool tem_flag      = false;
//            for(Fluent_Vec::const_iterator it_pre = pre.begin(); it_pre != pre.end(); it_pre++){
//                unsigned p = *it_pre;
//
//                if( m_graph->is_landmark(p) ){
//                    Landmarks_Graph::Node* n = m_graph->node(p);
//                    if( !n->is_consumed() )
//                        if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
//                            if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
//                            //std::cout << "\t -- "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
//                            n->consume( );
//                            keep_consumed->push_back( n->is_consumed_ptr() );
//
//                            fix_point_flag= true;
//                            tem_flag      =true;
//                            continue;
//                        }
//                }
//                if (!tem_flag)
//                    fix_point_flag= false;
//            }
//        }
////        for(Fluent_Vec::const_iterator it_pre = pre.begin(); it_pre != pre.end(); it_pre++){
////            unsigned p = *it_pre;
////
////            if( m_graph->is_landmark(p) ){
////                Landmarks_Graph::Node* n = m_graph->node(p);
////                if( !n->is_consumed() )
////                    if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
////                        if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
////                        //std::cout << "\t -- "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
////                        n->consume( );
////                        keep_consumed->push_back( n->is_consumed_ptr() );
////                    }
////            }
////        }
//
//        for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
//            unsigned p = *it_add;
//
//            if( m_graph->is_landmark(p) ){
//                bool unconsume = false;
//                Landmarks_Graph::Node* n = m_graph->node(p);
//
//                if( n->is_consumed() )
//                    /**chao edit change the m_strips_model.is_in_goal to m_strips_model.is_in_init
//                     *
//                     */
//                    //if(  m_strips_model.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
//                    if(  m_strips_model.is_in_init(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
//                        unconsume = true;
//
//
//                if(unconsume){
//                    //std::cout << "\t ++ "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
//                    if( !keep_unconsumed  ) keep_unconsumed = new Bool_Vec_Ptr;
//
//                    n->unconsume( );
//                    keep_unconsumed->push_back( n->is_consumed_ptr() );
//
//
//
//                }
//            }
//        }
#endif
        if( !a->ceff_vec().empty() ){
            for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
                Conditional_Effect* ce = a->ceff_vec()[i];
                if( ce->can_be_applied_on( *s ) )
                    apply_cond_eff_fwd(s,ce, keep_consumed, keep_unconsumed);
            }
        }

    }

	void apply_cond_eff_bwd( State* s, Conditional_Effect* ce, Bool_Vec_Ptr*& keep_consumed, Bool_Vec_Ptr*& keep_unconsumed ){

	  
		const Fluent_Vec& add = ce->add_vec();
		const Fluent_Vec& del = ce->del_vec();		

		for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
			unsigned p = *it_add;
			
			if( m_graph_bwd->is_landmark(p) ){
				Landmarks_Graph::Node* n = m_graph_bwd->node(p);
				if( !n->is_consumed() )
					if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
						if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
						//std::cout << "\t -- "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;														       
						n->consume( );
						keep_consumed->push_back( n->is_consumed_ptr() );
					}
			}
		}

		for(Fluent_Vec::const_iterator it_del = del.begin(); it_del != del.end(); it_del++){
			unsigned p = *it_del;
						
			if( m_graph_bwd->is_landmark(p) ){
				bool unconsume = false;
				Landmarks_Graph::Node* n = m_graph_bwd->node(p);

				if( n->is_consumed() )
					if(  m_strips_model_bwd.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
						unconsume = true;
				

				if(unconsume){
					//std::cout << "\t ++ "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;					
					if( !keep_unconsumed  ) keep_unconsumed = new Bool_Vec_Ptr;

					n->unconsume( );
					keep_unconsumed->push_back( n->is_consumed_ptr() );
					

				
				}
			}
		}
	}
    void apply_cond_eff_fwd( State* s, Conditional_Effect* ce, Bool_Vec_Ptr*& keep_consumed, Bool_Vec_Ptr*& keep_unconsumed ){


        const Fluent_Vec& add = ce->add_vec();
        const Fluent_Vec& del = ce->del_vec();

        for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
            unsigned p = *it_add;

            if( m_graph_fwd->is_landmark(p) ){
                Landmarks_Graph::Node* n = m_graph_fwd->node(p);
                if( !n->is_consumed() )
                    if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
                        if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
                        //std::cout << "\t -- "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
                        n->consume( );
                        keep_consumed->push_back( n->is_consumed_ptr() );
                    }
            }
        }

        for(Fluent_Vec::const_iterator it_del = del.begin(); it_del != del.end(); it_del++){
            unsigned p = *it_del;

            if( m_graph_fwd->is_landmark(p) ){
                bool unconsume = false;
                Landmarks_Graph::Node* n = m_graph_fwd->node(p);

                if( n->is_consumed() )
                    if(  m_strips_model_fwd.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                        unconsume = true;


                if(unconsume){
                    //std::cout << "\t ++ "<<p <<" - " << m_strips_model.fluents()[ p ]->signature() << std::endl;
                    if( !keep_unconsumed  ) keep_unconsumed = new Bool_Vec_Ptr;

                    n->unconsume( );
                    keep_unconsumed->push_back( n->is_consumed_ptr() );



                }
            }
        }
    }

	void apply_action_bwd( const State* s, Action_Idx a_idx ){
		const Action* a = m_strips_model_bwd.actions()[ a_idx ];
		/**orginal
		 *

		const Fluent_Vec& add = a->add_vec();
		const Fluent_Vec& del = a->del_vec();		

		for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
			unsigned p = *it_add;
			
			if( m_graph->is_landmark(p) ){
				Landmarks_Graph::Node* n = m_graph->node(p);
				if( !n->is_consumed() )
					if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
					    n->consume( );
					}
			}
		}

		for(Fluent_Vec::const_iterator it_del = del.begin(); it_del != del.end(); it_del++){
			unsigned p = *it_del;
						
			if( m_graph->is_landmark(p) ){
				bool unconsume = false;
				Landmarks_Graph::Node* n = m_graph->node(p);

				if( n->is_consumed() )
					if(  m_strips_model.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
						unconsume = true;
				

				if(unconsume){
					n->unconsume( );
				}
			}
		}
        */
		/**chao edit
		 *
		 */

        const Fluent_Vec& add = a->add_vec();
        const Fluent_Vec& pre = a->prec_vec();

        for(Fluent_Vec::const_iterator it_pre = pre.begin(); it_pre != pre.end(); it_pre++){
            unsigned p = *it_pre;

            if( m_graph_bwd->is_landmark(p) ){
                Landmarks_Graph::Node* n = m_graph_bwd->node(p);
                if( !n->is_consumed() )
                    if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
                        n->consume( );
                    }
            }
        }

        for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
            unsigned p = *it_add;

            if( m_graph_bwd->is_landmark(p) ){
                bool unconsume = false;
                Landmarks_Graph::Node* n = m_graph_bwd->node(p);
                /** chao edit
                 *  change m_strips_model.is_in_goal(p) to m_strips_model.is_in_init(p)
                 */
                if( n->is_consumed() )
                    //if(  m_strips_model.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                    if(  m_strips_model_bwd.is_in_init(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                        unconsume = true;


                if(unconsume){
                    n->unconsume( );
                }
            }
        }
		if( !a->ceff_vec().empty() ){
			for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
				Conditional_Effect* ce = a->ceff_vec()[i];
				if( ce->can_be_applied_on( *s ) )
					apply_cond_eff_bwd(s,ce);
			}
		}

	}

    void apply_action_fwd( const State* s, Action_Idx a_idx ){
        const Action* a = m_strips_model_fwd.actions()[ a_idx ];

        const Fluent_Vec& add = a->add_vec();
        const Fluent_Vec& del = a->del_vec();

        for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
            unsigned p = *it_add;

            if( m_graph_fwd->is_landmark(p) ){
                Landmarks_Graph::Node* n = m_graph_fwd->node(p);
                if( !n->is_consumed() )
                    if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
                        n->consume( );
                    }
            }
        }

        for(Fluent_Vec::const_iterator it_del = del.begin(); it_del != del.end(); it_del++){
            unsigned p = *it_del;

            if( m_graph_fwd->is_landmark(p) ){
                bool unconsume = false;
                Landmarks_Graph::Node* n = m_graph_fwd->node(p);

                if( n->is_consumed() )
                    if(  m_strips_model_fwd.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                        unconsume = true;


                if(unconsume){
                    n->unconsume( );
                }
            }
        }
//
//
//        const Fluent_Vec& add = a->add_vec();
//        const Fluent_Vec& pre = a->prec_vec();
//
//        for(Fluent_Vec::const_iterator it_pre = pre.begin(); it_pre != pre.end(); it_pre++){
//            unsigned p = *it_pre;
//
//            if( m_graph->is_landmark(p) ){
//                Landmarks_Graph::Node* n = m_graph->node(p);
//                if( !n->is_consumed() )
//                    if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
//                        n->consume( );
//                    }
//            }
//        }
//
//        for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
//            unsigned p = *it_add;
//
//            if( m_graph->is_landmark(p) ){
//                bool unconsume = false;
//                Landmarks_Graph::Node* n = m_graph->node(p);
//                /** chao edit
//                 *  change m_strips_model.is_in_goal(p) to m_strips_model.is_in_init(p)
//                 */
//                if( n->is_consumed() )
//                    //if(  m_strips_model.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
//                    if(  m_strips_model.is_in_init(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
//                        unconsume = true;
//
//
//                if(unconsume){
//                    n->unconsume( );
//                }
//            }
//        }
        if( !a->ceff_vec().empty() ){
            for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
                Conditional_Effect* ce = a->ceff_vec()[i];
                if( ce->can_be_applied_on( *s ) )
                    apply_cond_eff_fwd(s,ce);
            }
        }

    }


	void apply_cond_eff_bwd( const State* s, Conditional_Effect* ce ){

	  
		const Fluent_Vec& add = ce->add_vec();
		const Fluent_Vec& del = ce->del_vec();		

		for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
			unsigned p = *it_add;
			
			if( m_graph_bwd->is_landmark(p) ){
				Landmarks_Graph::Node* n = m_graph_bwd->node(p);
				if( !n->is_consumed() )
					if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
						n->consume( );
					}
			}
		}

		for(Fluent_Vec::const_iterator it_del = del.begin(); it_del != del.end(); it_del++){
			unsigned p = *it_del;
						
			if( m_graph_bwd->is_landmark(p) ){
				bool unconsume = false;
				Landmarks_Graph::Node* n = m_graph_bwd->node(p);
                /** chao edit
                  *  change m_strips_model.is_in_goal(p) to m_strips_model.is_in_init(p)
                 */
				if( n->is_consumed() )
                    if(  m_strips_model_bwd.is_in_init(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
					//if(  m_strips_model.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
						unconsume = true;
				

				if(unconsume){
					n->unconsume( );
				}
			}
		}
	}
    void apply_cond_eff_fwd( const State* s, Conditional_Effect* ce ){


        const Fluent_Vec& add = ce->add_vec();
        const Fluent_Vec& del = ce->del_vec();

        for(Fluent_Vec::const_iterator it_add = add.begin(); it_add != add.end(); it_add++){
            unsigned p = *it_add;

            if( m_graph_fwd->is_landmark(p) ){
                Landmarks_Graph::Node* n = m_graph_fwd->node(p);
                if( !n->is_consumed() )
                    if( n->are_precedences_consumed()  && n->are_gn_precedences_consumed() ){
                        n->consume( );
                    }
            }
        }

        for(Fluent_Vec::const_iterator it_del = del.begin(); it_del != del.end(); it_del++){
            unsigned p = *it_del;

            if( m_graph_fwd->is_landmark(p) ){
                bool unconsume = false;
                Landmarks_Graph::Node* n = m_graph_fwd->node(p);
                /** chao edit
                  *  change m_strips_model.is_in_goal(p) to m_strips_model.is_in_init(p)
                 */
                if( n->is_consumed() )
                    if(  m_strips_model_fwd.is_in_init(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                        //if(  m_strips_model.is_in_goal(p) || (! n->are_requirements_consumed() ) || (! n->are_gn_requirements_consumed() ) )
                        unconsume = true;


                if(unconsume){
                    n->unconsume( );
                }
            }
        }
    }

	void apply_state_bwd( const Fluent_Vec& fl, Bool_Vec_Ptr*& keep_consumed, Bool_Vec_Ptr*& keep_unconsumed ){
	    /** chao edit
	     *
	     */
        for(Fluent_Vec::const_iterator it_fl = fl.begin(); it_fl != fl.end(); it_fl++){
            unsigned p = *it_fl;
            if( m_graph_bwd->is_landmark(p) ){
                if( m_strips_model_bwd.is_in_goal(p) ){

                    Landmarks_Graph::Node* n = m_graph_bwd->node(p);
                    keep_consumed = new Bool_Vec_Ptr;
                    n->consume( );
                    keep_consumed->push_back( n->is_consumed_ptr() );
                }
            }
            }

		for(Fluent_Vec::const_iterator it_fl = fl.begin(); it_fl != fl.end(); it_fl++){
			unsigned p = *it_fl;

			if( m_graph_bwd->is_landmark(p) ){
				
				Landmarks_Graph::Node* n = m_graph_bwd->node(p);
				if( (! n->is_consumed()) && n->are_precedences_consumed() && n->are_gn_precedences_consumed() ){
					if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
					n->consume( );
					keep_consumed->push_back( n->is_consumed_ptr() );
				}
			}
		}

	}
    void apply_state_fwd( const Fluent_Vec& fl, Bool_Vec_Ptr*& keep_consumed, Bool_Vec_Ptr*& keep_unconsumed ){

        for(Fluent_Vec::const_iterator it_fl = fl.begin(); it_fl != fl.end(); it_fl++){
            unsigned p = *it_fl;

            if( m_graph_fwd->is_landmark(p) ){

                Landmarks_Graph::Node* n = m_graph_fwd->node(p);
                if( (! n->is_consumed()) && n->are_precedences_consumed() && n->are_gn_precedences_consumed() ){
                    if( !keep_consumed ) keep_consumed = new Bool_Vec_Ptr;
                    n->consume( );
                    keep_consumed->push_back( n->is_consumed_ptr() );
                }
            }
        }

    }

	void apply_state_bwd( const Fluent_Vec& fl ){

		for(Fluent_Vec::const_iterator it_fl = fl.begin(); it_fl != fl.end(); it_fl++){
			unsigned p = *it_fl;

			if( m_graph_bwd->is_landmark(p) ){
				
				Landmarks_Graph::Node* n = m_graph_bwd->node(p);
				if( (! n->is_consumed()) && n->are_precedences_consumed() && n->are_gn_precedences_consumed() ){
					n->consume( );	
				}
			}
		}

	}
    void apply_state_fwd( const Fluent_Vec& fl ){

        for(Fluent_Vec::const_iterator it_fl = fl.begin(); it_fl != fl.end(); it_fl++){
            unsigned p = *it_fl;

            if( m_graph_fwd->is_landmark(p) ){

                Landmarks_Graph::Node* n = m_graph_fwd->node(p);
                if( (! n->is_consumed()) && n->are_precedences_consumed() && n->are_gn_precedences_consumed() ){
                    n->consume( );
                }
            }
        }

    }
	
	
public:
	Landmarks_Graph*         graph_bwd(){ return m_graph_bwd; }
	const	STRIPS_Problem&	 problem_bwd() const			{ return m_strips_model_bwd; }

    Landmarks_Graph*         graph_fwd(){ return m_graph_fwd; }
    const	STRIPS_Problem&	 problem_fwd() const			{ return m_strips_model_fwd; }


protected:
    const bwd_Search_Model&           search_prob_bwd;
	const STRIPS_Problem&			m_strips_model_bwd;
	Landmarks_Graph*                         m_graph_bwd;

    const Fwd_Search_Model&           search_prob_fwd;
    const STRIPS_Problem&			m_strips_model_fwd;
    Landmarks_Graph*                         m_graph_fwd;
};

}

}

#endif // landmark_graph_manager.hxx
