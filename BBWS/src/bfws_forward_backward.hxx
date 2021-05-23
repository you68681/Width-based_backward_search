//
// Created by chao on 2020/8/29.
//

#ifndef SIW_FFPARSER_CMAKE_BFWS_FORWARD_BACKWARD_H
#define SIW_FFPARSER_CMAKE_BFWS_FORWARD_BACKWARD_H

#endif //SIW_FFPARSER_CMAKE_BFWS_FORWARD_BACKWARD_H

#include <aptk/search_prob.hxx>
#include <aptk/resources_control.hxx>
#include <aptk/closed_list.hxx>
#include <landmark_graph_manager.hxx>
#include <vector>
#include <algorithm>
#include <iostream>
#include <aptk/hash_table.hxx>
#include <landmark_graph_generator.hxx>
#include <novelty.hxx>


namespace aptk {

    namespace search {

        namespace bfws_forward_backward{


            template <typename bwd_Search_Model,typename Fwd_Search_Model, typename State>
            class Node {
            public:
                typedef aptk::agnostic::Landmarks_Graph_Manager<bwd_Search_Model,Fwd_Search_Model>   Landmarks_Graph_Manager;


                typedef State                            State_Type;
                typedef Node<bwd_Search_Model,Fwd_Search_Model,State>*        Node_Ptr;
                typedef typename std::vector< Node<bwd_Search_Model,Fwd_Search_Model,State>* >                      Node_Vec_Ptr;
                typedef typename std::vector< Node<bwd_Search_Model,Fwd_Search_Model,State>* >::reverse_iterator    Node_Vec_Ptr_Rit;
                typedef typename std::vector< Node<bwd_Search_Model,Fwd_Search_Model,State>* >::iterator            Node_Vec_Ptr_It;

                Node( State* s, float cost, Action_Idx action, Node<bwd_Search_Model,Fwd_Search_Model,State>* parent, int num_actions )
                        : m_state( s ), m_parent( parent ), m_action(action), m_g( 0 ), m_g_unit(0), m_h1(0), m_h2(0), m_novelty(0), m_r(0), m_partition(0), m_M(0), m_land_consumed(NULL), m_land_unconsumed(NULL), m_rp_fl_vec(NULL), m_rp_fl_set(NULL), m_relaxed_deadend(false) {
                    m_g = ( parent ? parent->m_g + cost : 0.0f);
                    m_g_unit = ( parent ? parent->m_g_unit + 1 : 0);
                }


                Node(Node &node){

                    m_state = new State (*(node.m_state));
                    m_rp_fl_vec = node.m_rp_fl_vec;

                    m_rp_fl_set = node.m_rp_fl_set;
                    m_parent=node.parent();


                }

                virtual ~Node() {
                    if ( m_state != NULL ) delete m_state;
                    if ( m_rp_fl_vec != NULL ) delete m_rp_fl_vec;
                    if ( m_rp_fl_set != NULL ) delete m_rp_fl_set;

                }

                unsigned&		h1n()				{ return m_h1; }
                unsigned		h1n() const 			{ return m_h1; }
                unsigned&		h2n()				{ return m_h2; }
                unsigned		h2n() const 			{ return m_h2; }
                /** chao edit
                 *
                 * @return
                 */
                unsigned&		hnovelty()				{ return m_novelty; }
                unsigned		hnovelty() const 			{ return m_novelty; }
                unsigned&		r()				{ return m_r; }
                unsigned		r() const 			{ return m_r; }
                unsigned&		M()				{ return m_M; }
                unsigned		M() const 			{ return m_M; }
                unsigned&      		partition()    	  { return m_partition; }
                unsigned       		partition() const { return m_partition; }

                float&			gn()				{ return m_g; }
                float			gn() const 			{ return m_g; }
                unsigned&      		gn_unit()			{ return m_g_unit; }
                unsigned       		gn_unit() const 		{ return m_g_unit; }

                Node_Ptr		parent()   			{ return m_parent; }
                const Node_Ptr		parent() const 			{ return m_parent; }
                Action_Idx		action() const 			{ return m_action; }
                State*			state()				{ return m_state; }
                void			set_state( State* s )  		{ m_state = s; }
                bool			has_state() const		{ return m_state != NULL; }
                const State&		state() const 			{ return *m_state; }
                Bool_Vec_Ptr*&          land_consumed()                 { return m_land_consumed; }
                Bool_Vec_Ptr*&          land_unconsumed()               { return m_land_unconsumed; }
                Fluent_Vec*&            rp_vec()                        { return m_rp_fl_vec; }
                Fluent_Set*&            rp_set()                        { return m_rp_fl_set; }
                bool&                   relaxed_deadend()               { return m_relaxed_deadend;}

                void        set_forward_head(Node* head_forward)     {m_node_forward=head_forward;}
                void         set_backward_head(Node* head_backward)   {m_node_backward=head_backward;}

                Node*        get_forward_head()     { return m_node_forward;}
                Node*         get_backward_head()   {return m_node_backward;}



                //Used to update novelty table
                bool                  is_better( Node* n ) const{


                    return false;

                    //One could mark as novel a tuple that has a better reward, like ICAPS17
                    //it is orthogonal, solves more problems in some domains, less in others.

                    //return this->gn() < n->gn();


                }

                void                    update_land_graph_bwd(Landmarks_Graph_Manager* lgm){
                    Node_Vec_Ptr path( gn_unit()+1 );
                    Node_Vec_Ptr_Rit rit = path.rbegin();
                    Node_Ptr n = this;

                    do{
                        *rit = n;
                        rit++;
                        n = n->parent();
                    }while( n );
                    if(rit != path.rend())
                        *rit = NULL;

                    /** check later
                     *
                     */
                    lgm->reset_graph_bwd();
                    for( Node_Vec_Ptr_It it = path.begin(); it != path.end(); it++){

                        if(*it == NULL) break;
                        lgm->update_graph( (*it)->land_consumed(), (*it)->land_unconsumed() );

                    }
                }

                void                    update_land_graph_fwd(Landmarks_Graph_Manager* lgm){
                    Node_Vec_Ptr path( gn_unit()+1 );
                    Node_Vec_Ptr_Rit rit = path.rbegin();
                    Node_Ptr n = this;

                    do{
                        *rit = n;
                        rit++;
                        n = n->parent();
                    }while( n );
                    if(rit != path.rend())
                        *rit = NULL;

                    /** check later
                     *
                     */
                    lgm->reset_graph_fwd();
                    for( Node_Vec_Ptr_It it = path.begin(); it != path.end(); it++){

                        if(*it == NULL) break;
                        lgm->update_graph( (*it)->land_consumed(), (*it)->land_unconsumed() );

                    }
                }

                void                    undo_land_graph( Landmarks_Graph_Manager* lgm ){
                    lgm->undo_graph( land_consumed(), land_unconsumed() );
                }

                void			print( std::ostream& os ) const {
                    os << "{@ = " << this << ", s = " << m_state << ", parent = " << m_parent << ", g(n) = ";
                    os << m_g << ", h1(n) = " << m_h1 << ", h2(n) = " << m_h2 << ", r(n) = " << m_r  << "}";
                }

                bool   	operator==( const Node<bwd_Search_Model,Fwd_Search_Model,State>& o ) const {

                    if( &(o.state()) != NULL && &(state()) != NULL)
                        return (const State&)(o.state()) == (const State&)(state());
                    /**
                     * Lazy
                     */
                    if  ( m_parent == NULL ) {
                        if ( o.m_parent == NULL ) return true;
                        return false;
                    }

                    if ( o.m_parent == NULL ) return false;

                    return (m_action == o.m_action) && ( *(m_parent->m_state) == *(o.m_parent->m_state) );
                }

                size_t      hash() const { return m_state ? m_state->hash() : m_hash; }

                void        update_hash() {
                    Hash_Key hasher;
                    hasher.add( m_action );
                    if ( m_parent != NULL )
                        hasher.add( m_parent->state()->fluent_vec() );
                    m_hash = (size_t)hasher;
                }

            public:

                State*		m_state;
                Node_Ptr	m_parent;
                Action_Idx	m_action;
                float		m_g;
                unsigned       	m_g_unit;
                unsigned	m_h1;
                unsigned	m_h2;
                unsigned	m_novelty;
                unsigned	m_r;
                unsigned        m_partition;
                unsigned        m_M;

                size_t		m_hash;
                Bool_Vec_Ptr*   m_land_consumed;
                Bool_Vec_Ptr*   m_land_unconsumed;
                Fluent_Vec*     m_rp_fl_vec;
                Fluent_Set*     m_rp_fl_set;

                Fluent_Vec      m_goals_achieved;
                Fluent_Vec      m_goal_candidates;

                bool            m_relaxed_deadend;
                Node*            m_node_forward= nullptr;
                Node*           m_node_backward= nullptr;
            };



            template <typename bwd_Search_Problem, typename Fwd_Search_Problem, typename First_Heuristic, typename Second_Heuristic, typename Relevant_Fluents_Heuristic, typename Open_List_Type >
            class BFWS_Forward_Backward {

            public:

            public:
                typedef	        typename Fwd_Search_Problem::State_Type		        State_Forward;
                typedef	        typename bwd_Search_Problem::State_Type		        State_Backward;

                typedef  	typename Open_List_Type::Node_Type		        Search_Node;
                typedef  	bwd_Search_Problem					bwd_Search_Model;
                typedef  	Fwd_Search_Problem					fwd_Search_Model;
                typedef  	typename Open_List_Type::Node_Type		        Node;

                typedef 	Closed_List< Search_Node >			        Closed_List_Type;

                typedef         aptk::agnostic::Landmarks_Graph_Manager<bwd_Search_Problem,Fwd_Search_Problem>   Landmarks_Graph_Manager;
                typedef         aptk::agnostic::Landmarks_Graph_Manager<bwd_Search_Problem,Fwd_Search_Problem>   Landmarks_Graph_Manager_Forward;
                typedef         aptk::agnostic::Landmarks_Graph_Generator<bwd_Search_Problem, Fwd_Search_Problem>     Gen_Lms;

                BFWS_Forward_Backward( 	const bwd_Search_Problem& search_problem, const Fwd_Search_Problem& search_problem_forward )
                        : m_problem_bwd( search_problem),m_problem_fwd(search_problem_forward), m_exp_count_bwd(0), m_exp_count_fwd(0),m_gen_count_bwd(0), m_gen_count_fwd(0),m_dead_end_count_bwd(0),m_dead_end_count_fwd(0),
                        m_open_repl_count_bwd(0),m_open_repl_count_fwd(0),m_max_depth_bwd( infty ),m_max_depth_fwd( infty ), m_max_novelty_bwd(1),m_max_novelty_fwd(1), m_time_budget_bwd(infty),  m_time_budget_fwd(infty),
                        m_lgm(NULL), m_max_h2n_bwd(no_such_index), m_max_h2n_fwd(no_such_index),m_max_r_bwd(no_such_index), m_max_r_fwd(no_such_index), m_verbose_bwd( true ), m_verbose_fwd( true ),
                        m_use_novelty_bwd(true), m_use_novelty_fwd(true),m_use_novelty_pruning_bwd(false), m_use_novelty_pruning_fwd(false),m_use_rp_bwd(true),m_use_rp_fwd(true), m_use_rp_from_init_only_bwd(false),m_use_rp_from_init_only_fwd(false) {
                    m_first_h = new First_Heuristic( search_problem,search_problem_forward);

                   // m_first_h_forward = new First_Heuristic( search_problem_forward );
                    m_second_h = new Second_Heuristic( search_problem ,search_problem_forward);
                  //  m_second_h_forward = new Second_Heuristic( search_problem_forward );
                    m_relevant_fluents_h = new Relevant_Fluents_Heuristic( search_problem,search_problem_forward);
//                    m_novelty = new Abstract_Novelty( search_problem);

                }


                virtual ~BFWS_Forward_Backward() {
                    for ( typename Closed_List_Type::iterator i = m_closed_bwd.begin();
                          i != m_closed_bwd.end(); i++ ) {
                        delete i->second;
                    }
                    while	(!m_open_bwd.empty() )
                    {
                        Search_Node* n = m_open_bwd.pop();
                        delete n;
                    }
                    m_closed_bwd.clear();

                    for ( typename Closed_List_Type::iterator i = m_closed_fwd.begin();
                          i != m_closed_fwd.end(); i++ ) {
                        delete i->second;
                    }
                    while	(!m_open_fwd.empty() )
                    {
                        Search_Node* n = m_open_fwd.pop();
                        delete n;
                    }
                    m_closed_fwd.clear();

                    delete m_first_h;
                    delete m_second_h;
                    delete m_relevant_fluents_h;


                }

                /**
                 * Set the relevant fluents from node n
                 * computing a relaxed plan, and marking the fluents
                 * added by actions in relaxed plan as relevant
                 */




                void    set_relplan_bwd( Search_Node* n, State* s ){


                    std::vector<Action_Idx>	po;
                    std::vector<Action_Idx>	rel_plan;
                    unsigned h = 0;


                    m_relevant_fluents_h->ignore_rp_h_value_bwd(true);
                    m_relevant_fluents_h->eval_bwd( *s, h, po, rel_plan  );

                    if( h == std::numeric_limits<unsigned>::max() )
                        n->relaxed_deadend() = true;


#ifdef DEBUG
                    for ( unsigned p = 0; p < this->problem().task().num_fluents(); p++ ) {
			if (!m_rp_h->is_relaxed_plan_relevant(p)) continue;
			n->rp_vec()->push_back( p );
			n->rp_set()->set( p );
		}

		std::cout << "rel_plan size: "<< rel_plan.size() << " "<<std::flush;
#endif
                    /**
                     * Reserve space
                     */
                    if( !n->rp_vec() ){
                        n->rp_vec() = new Fluent_Vec;
                        n->rp_set() = new Fluent_Set( this->problem_backward().task().num_fluents() );
                    }
                    else{
                        n->rp_vec()->clear();
                        n->rp_set()->reset();

                    }

                    for(std::vector<Action_Idx>::iterator it_a = rel_plan.begin();
                        it_a != rel_plan.end(); it_a++ ){
                        const Action* a = this->problem_backward().task().actions()[*it_a];

                        //Add Conditional Effects
                        if( !a->ceff_vec().empty() ){
                            for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
                                Conditional_Effect* ce = a->ceff_vec()[i];
                                for ( auto p : ce->add_vec() ) {
                                    if ( ! n->rp_set()->isset( p ) ){
                                        n->rp_vec()->push_back( p );
                                        n->rp_set()->set( p );
#ifdef DEBUG
                                        std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
                                    }
                                }
                            }
                        }

                        const Fluent_Vec& pre = a->prec_vec();

#ifdef DEBUG
                        std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
#endif
                        for ( unsigned i = 0; i < pre.size(); i++ )
                        {
                            if ( ! n->rp_set()->isset( pre[i] ) )
                            {
                                n->rp_vec()->push_back( pre[i] );
                                n->rp_set()->set( pre[i] );
#ifdef DEBUG
                                std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
                            }
                        }

/*
			const Fluent_Vec& add = a->add_vec();

#ifdef DEBUG
			std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
#endif
			for ( unsigned i = 0; i < add.size(); i++ )
			{
				if ( ! n->rp_set()->isset( add[i] ) )
				{
					n->rp_vec()->push_back( add[i] );
					n->rp_set()->set( add[i] );
#ifdef DEBUG
					std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
				}
			}
*/
                    }
                }

                /** changed
                 *
                 * @param n
                 * @param s
                 */
                void    set_relplan_fwd( Search_Node* n, State* s ){


                    std::vector<Action_Idx>	po;
                    std::vector<Action_Idx>	rel_plan;
                    unsigned h = 0;
                    State* head_state;
                    if (n->parent()!=NULL){
                        if (n->parent()->get_backward_head()){
                            head_state=(n->parent()->get_backward_head()->state());
                        }
                        else{
                            head_state= nullptr;
                        }
                    }
                    else{
                        head_state= nullptr;
                    }



                    m_relevant_fluents_h->ignore_rp_h_value_fwd(true);
                    m_relevant_fluents_h->eval_fwd( *s, h, po, rel_plan,head_state);

                    if( h == std::numeric_limits<unsigned>::max() ) //rel_plan infty
                        n->relaxed_deadend() = true;


#ifdef DEBUG
                    for ( unsigned p = 0; p < this->problem().task().num_fluents(); p++ ) {
			if (!m_rp_h->is_relaxed_plan_relevant(p)) continue;
			n->rp_vec()->push_back( p );
			n->rp_set()->set( p );
		}

		std::cout << "rel_plan size: "<< rel_plan.size() << " "<<std::flush;
#endif
                    /**
                     * Reserve space
                     */
                    if( !n->rp_vec() ){
                        n->rp_vec() = new Fluent_Vec;
                        n->rp_set() = new Fluent_Set( this->problem_forward().task().num_fluents() );
                    }
                    else{
                        n->rp_vec()->clear();
                        n->rp_set()->reset();

                    }

                    for(std::vector<Action_Idx>::iterator it_a = rel_plan.begin();
                        it_a != rel_plan.end(); it_a++ ){
                        const Action* a = this->problem_forward().task().actions()[*it_a];

                        //Add Conditional Effects
                        if( !a->ceff_vec().empty() ){
                            for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
                                Conditional_Effect* ce = a->ceff_vec()[i];
                                for ( auto p : ce->add_vec() ) {
                                    if ( ! n->rp_set()->isset( p ) ){
                                        n->rp_vec()->push_back( p );
                                        n->rp_set()->set( p );
#ifdef DEBUG
                                        std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
                                    }
                                }
                            }
                        }

                        const Fluent_Vec& add = a->add_vec();

#ifdef DEBUG
                        std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
#endif
                        for ( unsigned i = 0; i < add.size(); i++ )
                        {
                            if ( ! n->rp_set()->isset( add[i] ) )
                            {
                                n->rp_vec()->push_back( add[i] );
                                n->rp_set()->set( add[i] );
#ifdef DEBUG
                                std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
                            }
                        }
                    }
                }

                /**
                 * chao edit
                 * @param n
                 * @param s
                 */
                void    set_relplan_edit( Search_Node* n, State* s,Fluent_Vec unachived_goal  ){

#ifdef  DEBUG
//            if (n->h2n()==0){
//        std::cout<<"relax plan find"<<std::endl;
//        s->print(std::cout);
//             }
#endif

                    bool flag= true;
                    std::vector<Action_Idx>	po;
                    std::vector<Action_Idx>	rel_plan;
                    unsigned h = 0;
                    /** check later
                     *
                     */
                    State* head_state;
                    if(n->parent()!=NULL) {
                        if (n->parent()->get_forward_head()){
                            head_state=(n->parent()->get_forward_head()->state());
                        }
                        else{
                            head_state= nullptr;
                        }
                    }
                    else{
                        head_state= nullptr;
                    }

                    m_relevant_fluents_h->ignore_rp_h_value_bwd(true);
                    m_relevant_fluents_h->eval_bwd( *s, h, po, rel_plan,flag , unachived_goal,head_state );

                    if( h == std::numeric_limits<unsigned>::max() )
                        n->relaxed_deadend() = true;


#ifdef DEBUG
                    for ( unsigned p = 0; p < this->problem().task().num_fluents(); p++ ) {
			if (!m_rp_h->is_relaxed_plan_relevant(p)) continue;
			n->rp_vec()->push_back( p );
			n->rp_set()->set( p );
		}

		std::cout << "rel_plan size: "<< rel_plan.size() << " "<<std::flush;
#endif
                    /**
                     * Reserve space
                     */
                    if( !n->rp_vec() ){
                        n->rp_vec() = new Fluent_Vec;
                        n->rp_set() = new Fluent_Set( this->problem_backward().task().num_fluents() );
                    }
                    else{
                        n->rp_vec()->clear();
                        n->rp_set()->reset();

                    }

                    for(std::vector<Action_Idx>::iterator it_a = rel_plan.begin();
                        it_a != rel_plan.end(); it_a++ ){
                        const Action* a = this->problem_backward().task().actions()[*it_a];

                        //Add Conditional Effects
                        if( !a->ceff_vec().empty() ){
                            for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
                                Conditional_Effect* ce = a->ceff_vec()[i];
                                for ( auto p : ce->add_vec() ) {
                                    if ( ! n->rp_set()->isset( p ) ){
                                        n->rp_vec()->push_back( p );
                                        n->rp_set()->set( p );
#ifdef DEBUG
                                        std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
                                    }
                                }
                            }
                        }

                        const Fluent_Vec& pre = a->prec_vec();

#ifdef DEBUG
                        std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
#endif
                        for ( unsigned i = 0; i < pre.size(); i++ )
                        {
                            if ( ! n->rp_set()->isset( pre[i] ) )
                            {
                                n->rp_vec()->push_back( pre[i] );
                                n->rp_set()->set( pre[i] );
#ifdef DEBUG
                                std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
                            }
                        }

/*
			const Fluent_Vec& add = a->add_vec();

#ifdef DEBUG
			std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
#endif
			for ( unsigned i = 0; i < add.size(); i++ )
			{
				if ( ! n->rp_set()->isset( add[i] ) )
				{
					n->rp_vec()->push_back( add[i] );
					n->rp_set()->set( add[i] );
#ifdef DEBUG
					std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
				}
			}
*/
                    }
                }

                virtual void	start_bwd( float B = infty) {
                    m_max_depth_bwd = B;
                    m_root_bwd = new Search_Node( m_problem_bwd.init_state(), 0.0f, no_op, NULL, m_problem_bwd.num_actions() );


                    //Init Novelty
                    m_first_h->init_bwd();

                    if(m_use_rp_bwd)
#if 0
                        /**
                         * original
                         */
                        //set_relplan( this->m_root, this->m_root->state() );
                        /** chao edi
                         */
#endif
                        m_root_edit_bwd = new Search_Node(m_problem_bwd.goal_state(), 0.0f, no_op, NULL, m_problem_bwd.num_actions());

                    set_relplan_bwd( this->m_root_bwd, this->m_root_edit_bwd->state() );

                    //if using the landmark manager to count goals or landmarks
                    if(m_lgm){
                        m_lgm->apply_state_bwd( m_root_bwd->state()->fluent_vec(), m_root_bwd->land_consumed(), m_root_bwd->land_unconsumed() );

                        eval_bwd(m_root_bwd);

                        if(m_use_rp_bwd)
                            eval_relevant_fluents_bwd(m_root_bwd);

                        if(m_use_novelty_bwd)
                            eval_novel_bwd( m_root_bwd );

                        m_root_bwd->undo_land_graph( m_lgm );
                    }
                    else{
                        eval_bwd(m_root_bwd);

                        if(m_use_rp_bwd)
                            eval_relevant_fluents_bwd(m_root_bwd);

                        if(m_use_novelty_bwd)
                            eval_novel_bwd( m_root_bwd );

                    }

                    if( m_root_bwd->relaxed_deadend() ){ //rel_plan infty
#ifdef DEBUG
                        if ( m_verbose ) {
				std::cout << "h_add is infinite" << std::endl;
			}
#endif
                        inc_dead_end_bwd();
                        return;;
                    }




#ifdef DEBUG
                    if ( m_verbose ) {
			std::cout << "Initial search node: ";
			m_root->print(std::cout);
			std::cout << std::endl;
			m_root->state()->print( std::cout );
			std::cout << std::endl;
		}
#endif
                    m_open_bwd.insert( m_root_bwd );

                    inc_gen_bwd();
                }

                /**
                 * changed
                 * @param B
                 */
                virtual void	start_fwd( float B = infty) {
                    m_max_depth_fwd = B;
                    m_root_fwd = new Search_Node( m_problem_fwd.init_state(), 0.0f, no_op, NULL, m_problem_fwd.num_actions() );
                    //Init Novelty
                    m_first_h->init_fwd();

                    if(m_use_rp_fwd)
                        set_relplan_fwd( this->m_root_fwd, this->m_root_fwd->state() );

                    //if using the landmark manager to count goals or landmarks
                    if(m_lgm){
                        m_lgm->apply_state_fwd( m_root_fwd->state()->fluent_vec(), m_root_fwd->land_consumed(), m_root_fwd->land_unconsumed() );

                        eval_fwd(m_root_fwd);

                        if(m_use_rp_fwd)
                            eval_relevant_fluents_fwd(m_root_fwd);

                        if(m_use_novelty_fwd)
                            eval_novel_fwd( m_root_fwd );

                        m_root_fwd->undo_land_graph( m_lgm );
                    }
                    else{
                        eval_fwd(m_root_fwd);

                        if(m_use_rp_fwd)
                            eval_relevant_fluents_fwd(m_root_fwd);

                        if(m_use_novelty_fwd)
                            eval_novel_fwd( m_root_fwd );

                    }

                    if( m_root_fwd->relaxed_deadend() ){ //rel_plan infty
#ifdef DEBUG
                        if ( m_verbose ) {
				std::cout << "h_add is infinite" << std::endl;
			}
#endif
                        inc_dead_end_fwd();
                        return;;
                    }




#ifdef DEBUG
                    if ( m_verbose ) {
			std::cout << "Initial search node: ";
			m_root->print(std::cout);
			std::cout << std::endl;
			m_root->state()->print( std::cout );
			std::cout << std::endl;
		}
#endif
                    m_open_fwd.insert( m_root_fwd );

                    inc_gen_fwd();
                }





                virtual void      eval_bwd( Search_Node* candidate ) {
                    Fluent_Vec unachived_goal;


                    if(m_lgm){
                        //Update land/goal counter up to parent node
                        if(candidate->parent())
                            candidate->parent()->update_land_graph_bwd( m_lgm );

                        //Update counter with current operator
                        if (candidate->action() != no_op){
                            const bool has_cond_eff = !(m_problem_bwd.task().actions()[ candidate->action() ]->ceff_vec().empty());

                            //If state hasn't been generated yet, update counter progressing the state of the parent
                            if( !candidate->has_state() && has_cond_eff ){

                                candidate->parent()->state()->progress_lazy_state_bwd(  m_problem_bwd.task().actions()[ candidate->action() ] );

                                /**
                                 * cheak later
                                 */
                                m_lgm->apply_action_bwd( candidate->parent()->state(), candidate->action(), candidate->land_consumed(), candidate->land_unconsumed() );

                                candidate->parent()->state()->regress_lazy_state_bwd( m_problem_bwd.task().actions()[ candidate->action() ] );

                            }else{
                                //update the counter with current state
                                /**
                                 * cheak later
                                 */
                                m_lgm->apply_action_bwd( candidate->state(), candidate->action(), candidate->land_consumed(), candidate->land_unconsumed() );
                            }
                        }
                        else //If it's the root node, just initialize the counter
                                     /**
                                     * cheak later
                                     */
                            m_lgm->apply_state_bwd( m_root_bwd->state()->fluent_vec(), m_root_bwd->land_consumed(), m_root_bwd->land_unconsumed() );
                    }

                    //Count land/goal unachieved
                    /** chao edit
                     *
                     */

                    if (candidate->state()==NULL){
                        static Fluent_Vec added, deleted;
                        added.clear(); deleted.clear();
                        candidate->parent()->state()->progress_lazy_state_bwd(  m_problem_bwd.task().actions()[ candidate->action() ] );
                        /** check later
                         *
                         */
                        m_second_h->eval_bwd( *(candidate->state()), candidate->h2n(), (candidate->parent()->state()->fluent_vec()));
                        candidate->parent()->state()->regress_lazy_state_bwd(  this->problem_backward().task().actions()[ candidate->action() ], &added, &deleted );
                    }
                    if (candidate->state()!=NULL)
                        m_second_h->eval_bwd( *(candidate->state()), candidate->h2n(),candidate->state()->fluent_vec());


#if 0
//        m_second_h->eval( *(candidate->state()), candidate->h2n());




//        unachived_goal= m_second_h->eval_goal_count(*(candidate->state()), unachived_goal);

//        if (unachived_goal.empty()){
//            std::cout<<"find"<<std::endl;
//        }
#endif

                    //If relevant fluents are in use
                    if(m_use_rp_bwd && !m_use_rp_from_init_only_bwd){
                        //if land/goal counter has decreased, then update relevant fluents
                        if(candidate->parent() && candidate->h2n() < candidate->parent()->h2n() ){
                            //If state hasn't been gereated, update the parent state with current op
                            if( ! candidate->has_state() ){
                                static Fluent_Vec added, deleted;
                                added.clear(); deleted.clear();
                                candidate->parent()->state()->progress_lazy_state_bwd(  this->problem_backward().task().actions()[ candidate->action() ], &added, &deleted  );
# if 0
                                /**original
                                 *
                                 */
                                //set_relplan( candidate, candidate->parent()->state() );
                                /**chao edit
                                 *
                                 */
#endif
                                set_relplan_edit(candidate, candidate->parent()->state(),unachived_goal);
                                candidate->parent()->state()->regress_lazy_state_bwd(  this->problem_backward().task().actions()[ candidate->action() ], &added, &deleted );
                            }
                            else
                                set_relplan_bwd( candidate, candidate->state() );

                        }
                    }

                    if(candidate->h2n() < m_max_h2n_bwd ){
                        m_max_h2n_bwd = candidate->h2n();
                        m_max_r_bwd = 0;
                        if ( m_verbose_bwd ) {
#if 0
//                            static Fluent_Vec added, deleted;
//                            added.clear(); deleted.clear();
//                            if (candidate->action()!=-1){
//                                problem_backward().task().actions()[candidate->action()]->print(problem_backward().task(),std::cout);
//                            }
//                            if (candidate->parent()!=NULL){
//                                candidate->parent()->state()->progress_lazy_state_bwd(  this->problem_backward().task().actions()[ candidate->action() ], &added, &deleted  );
//                                candidate->parent()->state()->print(std::cout);
//                                candidate->parent()->state()->regress_lazy_state_bwd(  this->problem_backward().task().actions()[ candidate->action() ], &added, &deleted );
//                            }

                            //candidate->parent()->state()->print(std::cout);
//                if (m_max_h2n==0)
//                {
//                    std::cout<<'find'<<std::endl;
//                }
//                m_second_h->eval_edit( *(candidate->state()));
#endif
                            std::cout << "--[" << m_max_h2n_bwd<<" / " << m_max_r_bwd <<"]--" << std::endl;
                        }
                    }

                }

                virtual void      eval_fwd( Search_Node* candidate ) {


                    if(m_lgm){
                        //Update land/goal counter up to parent node
                        if(candidate->parent())
                            candidate->parent()->update_land_graph_fwd( m_lgm );

                        //Update counter with current operator
                        if (candidate->action() != no_op){
                            const bool has_cond_eff = !(m_problem_fwd.task().actions()[ candidate->action() ]->ceff_vec().empty());

                            //If state hasn't been generated yet, update counter progressing the state of the parent
                            if( !candidate->has_state() && has_cond_eff ){
                                candidate->parent()->state()->progress_lazy_state_fwd(  m_problem_fwd.task().actions()[ candidate->action() ] );

                                m_lgm->apply_action_fwd( candidate->parent()->state(), candidate->action(), candidate->land_consumed(), candidate->land_unconsumed() );

                                candidate->parent()->state()->regress_lazy_state_fwd( m_problem_fwd.task().actions()[ candidate->action() ] );

                            }else{
                                //update the counter with current state
                                m_lgm->apply_action_fwd( candidate->state(), candidate->action(), candidate->land_consumed(), candidate->land_unconsumed() );
                            }
                        }
                        else //If it's the root node, just initialize the counter
                            m_lgm->apply_state_fwd( m_root_fwd->state()->fluent_vec(), m_root_fwd->land_consumed(), m_root_fwd->land_unconsumed() );
                    }

                    //Count land/goal unachieved
                    m_second_h->eval_fwd( *(candidate->state()), candidate->h2n());


                    //If relevant fluents are in use
                    if(m_use_rp_fwd && !m_use_rp_from_init_only_fwd){
                        //if land/goal counter has decreased, then update relevant fluents
                        if(candidate->parent() && candidate->h2n() < candidate->parent()->h2n() ){
                            //If state hasn't been gereated, update the parent state with current op
                            if( ! candidate->has_state() ){
                                static Fluent_Vec added, deleted;
                                added.clear(); deleted.clear();
                                candidate->parent()->state()->progress_lazy_state_fwd(  this->problem_forward().task().actions()[ candidate->action() ], &added, &deleted  );
                                set_relplan_fwd( candidate, candidate->parent()->state() );
                                candidate->parent()->state()->regress_lazy_state_fwd(  this->problem_forward().task().actions()[ candidate->action() ], &added, &deleted );
                            }
                            else
                                set_relplan_fwd( candidate, candidate->state() );

                        }
                    }

                    if(candidate->h2n() < m_max_h2n_fwd ){
                        m_max_h2n_fwd = candidate->h2n();
                        m_max_r_fwd = 0;
                        if ( m_verbose_fwd ) {
                            std::cout << "--[" << m_max_h2n_fwd<<" / " << m_max_r_fwd <<"]--" << std::endl;
                        }
                    }

                }


                unsigned  rp_fl_achieved_bwd( Search_Node* n ){
                    unsigned count = 0;
                    static Fluent_Set counted( this->problem_backward().task().num_fluents() );
                    Search_Node* n_start = n;
                    while( !n_start->rp_vec() ){
                        n_start = n_start->parent();
                    }

                    while( n->action()!= no_op && n != n_start ){

                        const Action* a = this->problem_backward().task().actions()[ n->action() ];

                        //Add Conditional Effects
                        if( !a->ceff_vec().empty() ){
                            for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
                                Conditional_Effect* ce = a->ceff_vec()[i];
                                for ( auto p : ce->add_vec() ) {
                                    if( n_start->rp_set()->isset( p ) && ! counted.isset(p) ){
                                        count++;
                                        counted.set( p );
                                    }
                                }
                            }
                        }


                        const  Fluent_Vec & pre=a->prec_vec();
                        for (unsigned i=0; i<pre.size();i++){
                            const unsigned  p=pre[i];
                            if (n_start->rp_set()->isset(p) && !counted.isset(p))
                            {
//			        if(n->h2n()==9){
//                        std::cout<<"====prec==="<<std::endl;
//			        }
                                count++;
                                counted.set(p);
//                    if(n->h2n()==9){
//                        std::cout<< count<<problem().task().fluents()[p]->signature()<<std::endl;
//                    }
                            }
                        }

/*
               const Fluent_Vec& add = a->add_vec();

		       //std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
		       for ( unsigned i = 0; i < add.size(); i++ ){
			       const unsigned p = add[i];
			       if( n_start->rp_set()->isset( p ) && ! counted.isset(p) ){
				       count++;
				       counted.set( p );
			       }
		       }
*/
                        n = n->parent();
                    }
                    counted.reset();
                    return count;

                }

                unsigned  rp_fl_achieved_fwd( Search_Node* n ){
                    unsigned count = 0;
                    static Fluent_Set counted( this->problem_forward().task().num_fluents() );
                    Search_Node* n_start = n;
                    while( !n_start->rp_vec() ){
                        n_start = n_start->parent();
                    }

                    while( n->action()!= no_op && n != n_start ){

                        const Action* a = this->problem_forward().task().actions()[ n->action() ];

                        //Add Conditional Effects
                        if( !a->ceff_vec().empty() ){
                            for( unsigned i = 0; i < a->ceff_vec().size(); i++ ){
                                Conditional_Effect* ce = a->ceff_vec()[i];
                                for ( auto p : ce->add_vec() ) {
                                    if( n_start->rp_set()->isset( p ) && ! counted.isset(p) ){
                                        count++;
                                        counted.set( p );
                                    }
                                }
                            }
                        }

                        const Fluent_Vec& add = a->add_vec();

                        //std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
                        for ( unsigned i = 0; i < add.size(); i++ ){
                            const unsigned p = add[i];
                            if( n_start->rp_set()->isset( p ) && ! counted.isset(p) ){
                                count++;
                                counted.set( p );
                            }
                        }

                        n = n->parent();
                    }
                    counted.reset();
                    return count;

                }



                void			eval_relevant_fluents_bwd( Search_Node* candidate ) {
                    candidate->r() = rp_fl_achieved_bwd( candidate );

                    if(candidate->r() > m_max_r_bwd ){
                        //rp_fl_achieved_eidt(candidate);
//            static Fluent_Vec added, deleted;
//            added.clear(); deleted.clear();
//            if (candidate->parent()!=NULL){
//                candidate->parent()->state()->progress_lazy_state(  this->problem().task().actions()[ candidate->action() ], &added, &deleted  );
//                candidate->parent()->state()->print(std::cout);
//                candidate->parent()->state()->regress_lazy_state(  this->problem().task().actions()[ candidate->action() ], &added, &deleted );
//            }

                        m_max_r_bwd = candidate->r();

                        if ( m_verbose_bwd )
                            std::cout << "--[" << m_max_h2n_bwd <<" / " << m_max_r_bwd <<"]--" << std::endl;

                    }

                }

                void			eval_relevant_fluents_fwd( Search_Node* candidate ) {
                    candidate->r() = rp_fl_achieved_fwd( candidate );

                    if(candidate->r() > m_max_r_fwd ){
                        //rp_fl_achieved_eidt(candidate);
//            static Fluent_Vec added, deleted;
//            added.clear(); deleted.clear();
//            if (candidate->parent()!=NULL){
//                candidate->parent()->state()->progress_lazy_state(  this->problem().task().actions()[ candidate->action() ], &added, &deleted  );
//                candidate->parent()->state()->print(std::cout);
//                candidate->parent()->state()->regress_lazy_state(  this->problem().task().actions()[ candidate->action() ], &added, &deleted );
//            }

                        m_max_r_fwd = candidate->r();

                        if ( m_verbose_fwd )
                            std::cout << "--[" << m_max_h2n_fwd  <<" / " << m_max_r_fwd <<"]--" << std::endl;

                    }

                }

//                float   prune_fwd( Search_Node* n ){
//
//                    float node_novelty = infty;
//                    aptk::agnostic::Novelty<fwd_Search_Model,Search_Node>::eval( n, node_novelty );
//                    return node_novelty;
//                }
//                float   prune_bwd( Search_Node* n ){
//
//                    float node_novelty = infty;
//                    aptk::agnostic::Novelty<bwd_Search_Model,Search_Node>::eval( n, node_novelty );
//                    return node_novelty;
//                }

                void			eval_novel_bwd( Search_Node* candidate ) {
                    candidate->partition() = (1000 * candidate->h2n() )+ candidate->r();
                    m_first_h->eval_bwd( candidate, candidate->h1n()  );
                    m_first_h->eval_novelty_bwd(candidate,candidate->hnovelty());

                }
                void			eval_novel_fwd( Search_Node* candidate ) {
                    candidate->partition() = (1000 * candidate->h2n() )+ candidate->r();
                    m_first_h->eval_fwd( candidate, candidate->h1n()  );
                    //candidate->hnovelty();
                    m_first_h->eval_novelty_fwd(candidate, candidate->hnovelty());

                }


                bool 		is_closed_bwd( Search_Node* n ) 	{
                    Search_Node* n2 = this->closed_bwd().retrieve(n);

                    if ( n2 != NULL ) {
                        if ( n2->gn() <= n->gn() ) {
                            // The node we generated is a worse path than
                            // the one we already found
                            return true;
                        }
                        // Otherwise, we put it into Open and remove
                        // n2 from closed
                        this->closed_bwd().erase( this->closed_bwd().retrieve_iterator( n2 ) );
                    }
                    return false;
                }
                bool 		is_closed_fwd( Search_Node* n ) 	{
                    Search_Node* n2 = this->closed_fwd().retrieve(n);

                    if ( n2 != NULL ) {
                        if ( n2->gn() <= n->gn() ) {
                            // The node we generated is a worse path than
                            // the one we already found
                            return true;
                        }
                        // Otherwise, we put it into Open and remove
                        // n2 from closed
                        this->closed_fwd().erase( this->closed_fwd().retrieve_iterator( n2 ) );
                    }
                    return false;
                }

                /*
                 * check later
                 */

                Search_Node* 		get_node_bwd() {
                    Search_Node *next = NULL;
                    if(! m_open_bwd.empty() ) {
                        next = m_open_bwd.pop();
                    }
                    return next;
                }
                Search_Node* 		get_node_fwd() {
                    Search_Node *next = NULL;
                    if(! m_open_fwd.empty() ) {
                        next = m_open_fwd.pop();
                    }
                    return next;
                }

                /*
                 * check later
                 */
                Node* 		get_node_fwd_pri() {
                    Node *next = NULL;
                    if(! m_closed_fwd_pri.empty_close() ) {
                        next = m_closed_fwd_pri.pop_close();
                    }
                    return next;
                }

                Node* 		get_node_bwd_pri() {
                    Node *next = NULL;
                    if(! m_closed_bwd_pri.empty_close() ) {
                        next = m_closed_bwd_pri.pop_close();
                    }
                    return next;
                }



                void	 	open_node_bwd( Search_Node *n ) {
                    m_open_bwd.insert(n);
                    inc_gen_bwd();
                }
                void	 	open_node_fwd( Search_Node *n ) {
                    m_open_fwd.insert(n);
                    inc_gen_fwd();
                }
                /** chao add
                 *
                 * @param node
                 */
                void                    set_negation_bwd(Search_Node *node){
                    m_in_negation_bwd.clear();
                    m_in_negation_bwd.resize(problem_backward().task().num_fluents(), false);
                    while (node->parent()){
                        for (unsigned p: problem_backward().task().negation()){
                            if (node->state()->entails(p)){
                                m_in_negation_bwd[ p ] = TRUE;
                            }

                        }
                        node=node->parent();

                    }
                }
                void                    set_negation_fwd(Search_Node *node){
                    m_in_negation_fwd.clear();
                    m_in_negation_fwd.resize(problem_forward().task().num_fluents(), false);
                    while (node->parent()){
                        for (unsigned p: problem_forward().task().negation()){
                            if (node->state()->entails(p)){
                                m_in_negation_fwd[ p ] = TRUE;
                            }

                        }
                        node=node->parent();

                    }
                }
                virtual void 			process_bwd(  Search_Node *head ) {


#ifdef DEBUG
                    if ( m_verbose ) {
			std::cout << "Expanding:" << std::endl;
			head->print(std::cout);
			std::cout << std::endl;
			head->state()->print( std::cout );
			std::cout << std::endl;
		}
#endif

                    if(m_lgm)
                        head->update_land_graph_bwd( m_lgm );
                    //get negation from parent state.
//		set_negation(head);
                    //get all negation
                    //m_in_negation=this->problem().task().get_negation();
                    std::vector< aptk::Action_Idx > app_set;
#ifdef DEBUG
//		this->problem().applicable_set_v2( *(head->state()), app_set );
//        if (head->h2n()==3){
//            std::cout<<"find"<<std::endl;
//        }
//        if (head->h2n()==13){
//            std::cout<<"find"<<std::endl;
//        }
#endif
                    for (unsigned i = 0; i < this->problem_backward().num_actions(); ++i ) {
                        /**original version
                         *
                         */
                        if (this->problem_backward().is_applicable( *(head->state()), i )){
                            /** chao edit
                             *
                             */
                            // ignore the negation impact
                            //if (this->problem().is_applicable_edit( *(head->state()), i,m_in_negation )){
                            app_set.push_back(i);
                        }
                    }

                    for (unsigned i = 0; i < app_set.size(); ++i ) {
                        int a = app_set[i];

                        //Lazy state generation
                        State *succ = nullptr;

#ifdef  DEGUB

                        //std::cout<<result<<std::endl;
//            head->state()->print(std::cout);
                        //	std::cout<<"VVVVVVVV"<<std::endl;
//            this->problem().task().actions()[ a ]->print(this->problem().task(),std::cout);
//            succ->print(std::cout);

#endif

                        bool is_mutex = false;
                        const Action* a_ptr=this->problem_backward().task().actions()[a];
                        for (auto p: head->state()->fluent_vec()){
                            for (auto new_p: a_ptr->prec_vec()){
                                if (a_ptr->add_set().isset(p)) continue;
#ifdef DEBUG
                                /** chao edit
                                 *
                                 */
                                //ignore the negation in mutex check
                                //if (m_in_negation[new_p] or m_in_negation[p]) continue;
#endif

                                /** check later
                                 *
                                 * @param n
                                 * @param h_val
                                 */
                                if (this->problem_backward().h2_bwd().is_mutex_bwd(p,new_p)){
                                    is_mutex= true;
                                    break;
                                }
                            }
                            if (is_mutex) break;

                        }

                        Search_Node* n= nullptr;
                        if (is_mutex){
                            continue;
                        } else{
                            n = new Search_Node( succ, m_problem_bwd.cost( *(head->state()), a ), a, head, m_problem_bwd.num_actions()  );
                        }


#ifdef DEBUG
                        if ( m_verbose ) {
				std::cout << "Successor:" << std::endl;
				n->print(std::cout);
				std::cout << std::endl;
				if(n->has_state())
				n->state()->print( std::cout );
				std::cout << std::endl;
			}
#endif

                        eval_bwd( n );
                        if( n->relaxed_deadend() ){ //rel_plan infty
#ifdef DEBUG
                            if ( m_verbose ) {
					std::cout << "h_add is infinite" << std::endl;
				}
#endif
                            inc_dead_end_bwd();
                            delete n;
                            continue;
                        }

                        if(m_use_rp_bwd)
                            eval_relevant_fluents_bwd(n);

                        if(m_use_novelty_bwd){
                            eval_novel_bwd(n);
                            if(m_use_novelty_pruning_bwd)
                                if( n->h1n() > m_max_novelty_bwd ){
#ifdef DEBUG
                                    if ( m_verbose ) {
						    std::cout << "h_add is infinite" << std::endl;
					    }
#endif
                                    inc_dead_end_bwd();
                                    delete n;
                                    continue;
                                }

                        }

#ifdef DEBUG
                        if ( m_verbose )
				std::cout << "Inserted into OPEN" << std::endl;
#endif
//                        n->m_abstract_novelty=prune_bwd(n);
                        open_node_bwd(n);


                    }
                    inc_eval_bwd();
                }



                virtual void 			process_fwd(  Search_Node *head ) {


#ifdef DEBUG
                    if ( m_verbose ) {
			std::cout << "Expanding:" << std::endl;
			head->print(std::cout);
			std::cout << std::endl;
			head->state()->print( std::cout );
			std::cout << std::endl;
		}
#endif

                    if(m_lgm)
                        head->update_land_graph_fwd( m_lgm );
 /** original version dug
  *
  */
//                    std::vector< aptk::Action_Idx > app_set;
//                    this->problem().applicable_set_v2( *(head->state()), app_set );
                    std::vector< aptk::Action_Idx > app_set;

                    for (unsigned i = 0; i < this->problem_forward().num_actions(); ++i ) {

                        if (this->problem_forward().is_applicable( *(head->state()), i )){

                            app_set.push_back(i);
                        }
                    }


                    for (unsigned i = 0; i < app_set.size(); ++i ) {
                        int a = app_set[i];

                        //Lazy state generation
                        State *succ = nullptr;

                        Search_Node* n = new Search_Node( succ, m_problem_fwd.cost( *(head->state()), a ), a, head, m_problem_fwd.num_actions()  );

#ifdef DEBUG
                        if ( m_verbose ) {
				std::cout << "Successor:" << std::endl;
				n->print(std::cout);
				std::cout << std::endl;
				if(n->has_state())
				n->state()->print( std::cout );
				std::cout << std::endl;
			}
#endif

                        eval_fwd( n );
                        if( n->relaxed_deadend() ){ //rel_plan infty
#ifdef DEBUG
                            if ( m_verbose ) {
					std::cout << "h_add is infinite" << std::endl;
				}
#endif
                            inc_dead_end_fwd();
                            delete n;
                            continue;
                        }

                        if(m_use_rp_fwd)
                            eval_relevant_fluents_fwd(n);

                        if(m_use_novelty_fwd){
                            eval_novel_fwd(n);
                            if(m_use_novelty_pruning_fwd)
                                if( n->h1n() > m_max_novelty_fwd ){
#ifdef DEBUG
                                    if ( m_verbose ) {
						    std::cout << "h_add is infinite" << std::endl;
					    }
#endif
                                    inc_dead_end_fwd();
                                    delete n;
                                    continue;
                                }

                        }

#ifdef DEBUG
                        if ( m_verbose )
				std::cout << "Inserted into OPEN" << std::endl;
#endif
                        open_node_fwd(n);


                    }
                    inc_eval_fwd();
                }


               // virtual Search_Node*	 	do_search()
                virtual std::pair<Search_Node*,Search_Node*> do_search(Gen_Lms* graph=NULL ){

                   Search_Node *head_backward_copy=NULL;
                   Search_Node *head_forward_copy=NULL;
                    Search_Node *head_backward = get_node_bwd();
                   Search_Node  *head_forward =  get_node_fwd() ;

                   if (m_problem_bwd.task().direction()=="f"){
                       head_backward_copy = new Search_Node(*(head_backward));
                       head_forward_copy= new Search_Node(*(head_forward));
                       head_forward->set_backward_head(head_backward_copy);
                       head_backward->set_forward_head(head_forward_copy);
                   }


//
//                    while (head_forward){


                    while(head_forward || head_backward) {
                        if (head_forward==NULL && head_backward==NULL){
                            break;
                        }

                            while (head_forward!=NULL){
                                if ( head_forward->gn() >= max_depth_fwd() )  {
                                    close_fwd(head_forward);
                                    close_fwd_pri(head_forward);
                                    head_forward= get_node_fwd();
                                    if (m_problem_bwd.task().direction() =="f"){
                                        if(head_forward!= nullptr && head_backward_copy!= nullptr){
                                        head_forward->set_backward_head(head_backward_copy);
                                        }

                                        if (head_forward==NULL && head_backward!=NULL){
                                        head_forward_copy= nullptr;

                                        head_backward->set_forward_head(nullptr);
#ifdef DEBUG
                                      head_forward_copy= get_node_fwd_pri();
#endif
                                        head_backward->set_forward_head(head_forward_copy);
                                        auto m_graph_bwd = this->m_lgm->graph_bwd();

                                        graph->compute_lm_graph_set_additive_bwd( *m_graph_bwd );

 //                                       continue;

#ifdef DEBUG
                                        auto m_graph_bwd = this->m_lgm->graph_bwd();

                                        graph->compute_lm_graph_set_additive_bwd( *m_graph_bwd, head_forward_copy->state() );
#endif
                                    }
                                    }
#ifdef DEGUB
                                    if (m_problem_bwd.task().direction() =="f"&&m_problem_bwd.task().intersection() =="head") {
                                        if (head_backward != NULL && head_backward->state() != NULL &&
                                            head_forward == NULL && head_forward_copy != NULL &&
                                            head_forward_copy->state() != NULL) {
                                            if ((*(head_forward_copy->state())).entails(
                                                    head_backward->state()->fluent_vec())) {
                                                close_fwd(head_forward_copy);
                                                close_bwd(head_backward);
                                                close_fwd_pri(head_forward_copy);
                                                close_bwd_pri(head_backward);
                                                return {head_backward, head_forward_copy};
                                            }

                                        }
                                    }
#endif

                                    continue;
                                }

                                //Generate state
                                if( ! head_forward->has_state() )
                                    head_forward->set_state( m_problem_fwd.next(*(head_forward->parent()->state()), head_forward->action()) );
                                if (m_problem_bwd.task().intersection() =="close") {
                                    if (head_forward != NULL && head_forward->state() != NULL) {
                                        Fluent_Vec &head_forward_vec = head_forward->state()->fluent_vec();
                                        sort(head_forward_vec.begin(), head_forward_vec.end());

                                        for (auto &I:m_closed_bwd) {

                                            std::sort(I.second->state()->fluent_vec().begin(),
                                                      I.second->state()->fluent_vec().end());

                                            int left, right;
                                            auto k1 = lower_bound(head_forward_vec.begin(), head_forward_vec.end(),
                                                                  I.second->state()->fluent_vec().front());
                                            if (k1 == head_forward_vec.end() || (k1 != head_forward_vec.end() &&
                                                                                 (*k1) !=
                                                                                 I.second->state()->fluent_vec().front()))
                                                continue;
                                            left = k1 - head_forward_vec.begin();
                                            k1 = lower_bound(head_forward_vec.begin(), head_forward_vec.end(),
                                                             I.second->state()->fluent_vec().back());

                                            if (k1 == head_forward_vec.end() || (k1 != head_forward_vec.end() &&
                                                                                 (*k1) !=
                                                                                 I.second->state()->fluent_vec().back()))
                                                continue;
                                            right = k1 - head_forward_vec.begin();

                                            int j = 0;
                                            auto &vec = (I.second->state()->fluent_vec());
                                            int i;
                                            for (i = left; i <= right;) {
                                                if (vec[j] == head_forward_vec[i]) {
                                                    ++i;
                                                    ++j;
                                                } else if (vec[j] < head_forward_vec[i]) break;
                                                else if (vec[j] > head_forward_vec[i]) ++i;
                                            }


                                            if (j == vec.size()) {
                                                return {I.second, head_forward};
                                            }
                                        }
                                    }
                                }
                                if (m_problem_bwd.task().intersection() =="random") {
                                    if (head_forward != NULL && head_forward->state() != NULL) {
                                        Fluent_Vec &head_forward_vec = head_forward->state()->fluent_vec();
                                        sort(head_forward_vec.begin(), head_forward_vec.end());

                                        int _len = m_closed_bwd1.size();


                                        srand(unsigned(time(NULL)));
                                        std::vector<Search_Node *> __q;

                                        for (auto it = m_closed_bwd.begin(); it != m_closed_bwd.end(); ++it) {
                                            (__q).push_back((*it).second);
                                            // (_q)[ans++] = it;
                                        }
                                        // Closed_List_Type _m_close_fwd_copy ;

                                        std::vector<Search_Node *> _m_close_bwd_copy;
                                        for (int i = 1; i <= _len; ++i) {
                                            int id = rand() % m_closed_bwd.size();
                                            _m_close_bwd_copy.push_back(__q[id]);
                                        }


                                        for (auto &(_i):_m_close_bwd_copy) {  // auto &I:m_closed_bwd1

                                            Search_Node *I = (_i);
                                            std::sort(I->state()->fluent_vec().begin(), I->state()->fluent_vec().end());


                                            int left, right;
                                            auto k1 = lower_bound(head_forward_vec.begin(), head_forward_vec.end(),
                                                                  I->state()->fluent_vec().front());
                                            if (k1 == head_forward_vec.end() || (k1 != head_forward_vec.end() &&
                                                                                 (*k1) !=
                                                                                 I->state()->fluent_vec().front()))
                                                continue;
                                            left = k1 - head_forward_vec.begin();
                                            k1 = lower_bound(head_forward_vec.begin(), head_forward_vec.end(),
                                                             I->state()->fluent_vec().back());

                                            if (k1 == head_forward_vec.end() || (k1 != head_forward_vec.end() &&
                                                                                 (*k1) !=
                                                                                 I->state()->fluent_vec().back()))
                                                continue;
                                            right = k1 - head_forward_vec.begin();

                                            int j = 0;
                                            auto &vec = (I->state()->fluent_vec());
                                            int i;
                                            for (i = left; i <= right;) {
                                                if (vec[j] == head_forward_vec[i]) {
                                                    ++i;
                                                    ++j;
                                                } else if (vec[j] < head_forward_vec[i]) break;
                                                else if (vec[j] > head_forward_vec[i]) ++i;
                                            }


                                            if (j == vec.size()) {
                                                return {I, head_forward};
                                            }
                                        }
                                    }
                                }

                                if (m_problem_bwd.task().intersection() =="novelty") {
                                    if (head_forward != NULL && head_forward->state() != NULL) {
                                        Fluent_Vec &head_forward_vec = head_forward->state()->fluent_vec();
                                        sort(head_forward_vec.begin(), head_forward_vec.end());

                                        for (auto &I:m_closed_bwd1) {
#if 0
                                            if (I.second->state()->entails(head_forward_vec)) {
                                                return {I.second, head_forward};
                                            }

                                        }
#endif

                                            std::sort(I.second->state()->fluent_vec().begin(),
                                                      I.second->state()->fluent_vec().end());

                                            int left, right;
                                            auto k1 = lower_bound(head_forward_vec.begin(), head_forward_vec.end(),
                                                                  I.second->state()->fluent_vec().front());
                                            if (k1 == head_forward_vec.end() || (k1 != head_forward_vec.end() &&
                                                                                 (*k1) !=
                                                                                 I.second->state()->fluent_vec().front()))
                                                continue;
                                            left = k1 - head_forward_vec.begin();
                                            k1 = lower_bound(head_forward_vec.begin(), head_forward_vec.end(),
                                                             I.second->state()->fluent_vec().back());

                                            if (k1 == head_forward_vec.end() || (k1 != head_forward_vec.end() &&
                                                                                 (*k1) !=
                                                                                 I.second->state()->fluent_vec().back()))
                                                continue;
                                            right = k1 - head_forward_vec.begin();

                                            int j = 0;
                                            auto &vec = (I.second->state()->fluent_vec());
                                            int i;
                                            for (i = left; i <= right;) {
                                                if (vec[j] == head_forward_vec[i]) {
                                                    ++i;
                                                    ++j;
                                                } else if (vec[j] < head_forward_vec[i]) break;
                                                else if (vec[j] > head_forward_vec[i]) ++i;
                                            }


                                            if (j == vec.size()) {
                                                return {I.second, head_forward};
                                            }
                                        }
                                    }
                                }
                                if (m_problem_bwd.task().direction() =="e"&&m_problem_bwd.task().intersection() =="head") {
                                    if (head_backward != NULL && head_backward->state() != NULL &&
                                        head_forward != NULL && head_forward->state() != NULL) {
                                        if ((*(head_forward->state())).entails(head_backward->state()->fluent_vec())) {
                                            close_fwd(head_forward);
                                            close_bwd(head_backward);
                                            close_fwd_pri(head_forward);
                                            close_bwd_pri(head_backward);
                                            set_max_depth_fwd(head_forward->gn());
                                            return {head_backward, head_forward};
                                        }

                                    }
                                }
                                if (m_problem_bwd.task().direction() =="f"&&m_problem_bwd.task().intersection() =="head") {
                                    if (head_backward == NULL && head_backward_copy != NULL &&
                                        head_backward_copy->state() != NULL && head_forward != NULL &&
                                        head_forward->state() != NULL) {
                                        if ((*(head_forward->state())).entails(
                                                head_backward_copy->state()->fluent_vec())) {
                                            close_fwd(head_forward);
                                            close_bwd(head_backward_copy);
                                            close_fwd_pri(head_forward);
                                            close_bwd_pri(head_backward_copy);
                                            return {head_backward_copy, head_forward};
                                        }

                                    }
                                }

                                if(m_problem_fwd.goal(*(head_forward->state()))) {
                                    close_fwd(head_forward);
                                    close_fwd_pri(head_forward);

                                    set_max_depth_fwd( head_forward->gn() );
                                    return {NULL,head_forward};
                                }
                                if ( (time_used() - m_t0_fwd ) > m_time_budget_fwd )
                                    return {NULL,NULL};

                                if ( is_closed_fwd( head_forward ) ) {
#ifdef DEBUG
                                    if ( m_verbose )
					std::cout << "Already in CLOSED" << std::endl;
#endif
                                    delete head_forward;
                                    head_forward = get_node_fwd();
                                    if (m_problem_bwd.task().direction() =="f") {
                                        if (head_forward != nullptr && head_backward_copy != nullptr) {
                                            head_forward->set_backward_head(head_backward_copy);
                                        }

                                        if (head_forward == NULL && head_backward != NULL) {
                                            head_forward_copy = nullptr;

                                            head_backward->set_forward_head(nullptr);
#ifdef DEBUG
                                            head_forward_copy = get_node_fwd_pri();
#endif

                                            head_backward->set_forward_head(head_forward_copy);
                                            auto m_graph_bwd = this->m_lgm->graph_bwd();

                                            graph->compute_lm_graph_set_additive_bwd( *m_graph_bwd );
#ifdef DEBUG
                                    continue;
                                            auto m_graph_bwd = this->m_lgm->graph_bwd();

                                            graph->compute_lm_graph_set_additive_bwd(*m_graph_bwd,
                                                                                     head_forward_copy->state());
#endif
                                       }
                                    }
#ifdef DEBUG
                                 if (m_problem_bwd.task().direction() =="f"&&m_problem_bwd.task().intersection() =="head") {
                                     if (head_backward != NULL && head_backward->state() != NULL &&
                                         head_forward == NULL && head_forward_copy != NULL &&
                                         head_forward_copy->state() != NULL) {
                                         if ((*(head_forward_copy->state())).entails(
                                                 head_backward->state()->fluent_vec())) {
                                             close_fwd(head_forward_copy);
                                             close_bwd(head_backward);
                                             close_fwd_pri(head_forward_copy);
                                             close_bwd_pri(head_backward);
                                             return {head_backward, head_forward_copy};
                                         }

                                     }
                                 }
#endif
                                    continue;
                                }
                                process_fwd(head_forward);
                                close_fwd(head_forward);
                                close_fwd_pri(head_forward);
                                close_fwd1(head_forward);
                                head_forward = get_node_fwd();

#ifdef DEBUG
                                std::cout<<"closed_fwd_1: "<<m_closed_fwd1.size()<<std::endl;
                                std::cout<<"closed_fwd: "<<m_closed_fwd.size()<<std::endl;
#endif
                                if (m_problem_bwd.task().direction() =="f") {
                                    if (head_forward == NULL && head_backward != NULL) {
                                        head_forward_copy = nullptr;

                                        head_backward->set_forward_head(nullptr);
#ifdef DEGUB
                                        head_forward_copy = get_node_fwd_pri();
#endif
                                        head_backward->set_forward_head(head_forward_copy);
                                        auto m_graph_bwd = this->m_lgm->graph_bwd();

                                        graph->compute_lm_graph_set_additive_bwd( *m_graph_bwd );
                                        continue;
#ifdef DEBUG
                                        auto m_graph_bwd = this->m_lgm->graph_bwd();

                                        graph->compute_lm_graph_set_additive_bwd(*m_graph_bwd,
                                                                                 head_forward_copy->state());
#endif
                                    }

                                    if (head_forward != NULL && !head_forward->has_state())
                                        head_forward->set_state(m_problem_fwd.next(*(head_forward->parent()->state()),
                                                                                   head_forward->action()));
#ifdef DEBUG
                                    //                                if (head_forward!=NULL && head_forward->state()!=NULL)
                                    //                                {
                                    //                                    Fluent_Vec &head_forward_vec=head_forward->state()->fluent_vec();
                                    //                                    sort(head_forward_vec.begin(),head_forward_vec.end());
                                    //
                                    //                                    for(auto &I:m_closed_bwd){
                                    //
                                    //                                        std::sort(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end());
                                    //
                                    //                                        int left,right;
                                    //                                        auto k1 =  lower_bound(head_forward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                                    //                                        if(k1==head_forward_vec.end() || ( k1!=head_forward_vec.end()  &&(*k1)!=I.second->state()->fluent_vec().front())) continue;
                                    //                                        left = k1 - head_forward_vec.begin();
                                    //                                        k1 = lower_bound(head_forward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().back()) ;
                                    //
                                    //                                        if(k1==head_forward_vec.end() || (  k1!=head_forward_vec.end() && (*k1)!=I.second->state()->fluent_vec().back())) continue;
                                    //                                        right = k1 - head_forward_vec.begin();
                                    //
                                    //                                        int j = 0;
                                    //                                        auto &vec = (I.second->state()->fluent_vec());
                                    //                                        int i;
                                    //                                        for(i=left;i<=right;){
                                    //                                            if(vec[j]==head_forward_vec[i]){
                                    //                                                ++i;
                                    //                                                ++j;
                                    //                                            }else if(vec[j]<head_forward_vec[i]) break;
                                    //                                            else if(vec[j]>head_forward_vec[i]) ++i;
                                    //                                        }
                                    //
                                    //
                                    //                                        if(j==vec.size()) {
                                    //                                            return {I.second,head_forward};
                                    //                                        }
                                    //                                    }
                                    //                                }
                                    //                                if (head_forward!=NULL && head_forward->state()!=NULL)
                                    //                                {
                                    //                                    Fluent_Vec &head_forward_vec=head_forward->state()->fluent_vec();
                                    //                                    sort(head_forward_vec.begin(),head_forward_vec.end());
                                    //
                                    //                                    for(auto &I:m_closed_bwd1){
                                    //
                                    //                                        std::sort(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end());
                                    //
                                    //                                        int left,right;
                                    //                                        auto k1 =  lower_bound(head_forward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                                    //                                        if(k1==head_forward_vec.end() || ( k1!=head_forward_vec.end()  &&(*k1)!=I.second->state()->fluent_vec().front())) continue;
                                    //                                        left = k1 - head_forward_vec.begin();
                                    //                                        k1 = lower_bound(head_forward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().back()) ;
                                    //
                                    //                                        if(k1==head_forward_vec.end() || (  k1!=head_forward_vec.end() && (*k1)!=I.second->state()->fluent_vec().back())) continue;
                                    //                                        right = k1 - head_forward_vec.begin();
                                    //
                                    //                                        int j = 0;
                                    //                                        auto &vec = (I.second->state()->fluent_vec());
                                    //                                        int i;
                                    //                                        for(i=left;i<=right;){
                                    //                                            if(vec[j]==head_forward_vec[i]){
                                    //                                                ++i;
                                    //                                                ++j;
                                    //                                            }else if(vec[j]<head_forward_vec[i]) break;
                                    //                                            else if(vec[j]>head_forward_vec[i]) ++i;
                                    //                                        }
                                    //
                                    //
                                    //                                        if(j==vec.size()) {
                                    //                                            return {I.second,head_forward};
                                    //                                        }
                                    //                                    }
                                    //                                }
                                    //                                if (head_backward!=NULL && head_backward->state()!=NULL && head_forward!=NULL && head_forward->state()!=NULL){
                                    //                                    if((*(head_forward->state())).entails(head_backward->state()->fluent_vec())) {
                                    //                                        close_fwd(head_forward);
                                    //                                        close_bwd(head_backward);
                                    //                                        close_fwd_pri(head_forward);
                                    //                                        close_bwd_pri(head_backward);
                                    //                                        set_max_depth_fwd( head_forward->gn() );
                                    //                                        return {head_backward,head_forward};
                                    //                                    }
                                    //
                                    //                                }
#endif
                                    if (head_forward != NULL && head_backward != NULL) {
                                        head_forward_copy = new Search_Node(*head_forward);
                                        head_backward->set_forward_head(head_forward_copy);

                                        auto m_graph_bwd = this->m_lgm->graph_bwd();


                                        graph->compute_lm_graph_set_additive_bwd(*m_graph_bwd, head_forward->state());

                                    }
                                }
#ifdef DEBUG
//                                if (head_backward==NULL && head_backward_copy!=NULL && head_backward_copy->state()!=NULL && head_forward!=NULL && head_forward->state()!=NULL){
//                                    if((*(head_forward->state())).entails(head_backward_copy->state()->fluent_vec())) {
//                                        close_fwd(head_forward);
//                                        close_bwd(head_backward_copy);
//                                        close_fwd_pri(head_forward);
//                                        close_bwd_pri(head_backward_copy);
//                                        return {head_backward_copy,head_forward};
//                                    }
//
//                                }
//                                if (head_backward!=NULL && head_backward->state()!=NULL && head_forward==NULL && head_forward_copy!=NULL && head_forward_copy->state()!=NULL){
//                                    if((*(head_forward_copy->state())).entails(head_backward->state()->fluent_vec())) {
//                                        close_fwd(head_forward_copy);
//                                        close_bwd(head_backward);
//                                        close_fwd_pri(head_forward_copy);
//                                        close_bwd_pri(head_backward);
//                                        return {head_backward,head_forward_copy};
//                                    }
//
//                                }
#endif
                                break;

                            }



                        while (head_backward!=NULL){
                            if ( head_backward->gn() >= max_depth_bwd() )  {
                                close_bwd(head_backward);
                                close_bwd_pri(head_backward);
                                head_backward = get_node_bwd();
                                if (m_problem_bwd.task().direction()=="f"){
                                    if (head_backward!= nullptr && head_forward_copy!= nullptr){
                                        head_backward->set_forward_head(head_forward_copy);
                                    }
                                    if (head_backward==NULL && head_forward!=NULL){
                                        head_backward_copy= nullptr;

                                        head_forward->set_backward_head(nullptr);
# if 0
                                        head_backward_copy= get_node_bwd_pri();
#endif

                                        head_forward->set_backward_head(head_backward_copy);
                                        auto m_graph_fwd = this->m_lgm->graph_fwd();

                                        graph->compute_lm_graph_set_additive_fwd( *m_graph_fwd);
#if 0
                                    auto m_graph_fwd = this->m_lgm->graph_fwd();

                                    graph->compute_lm_graph_set_additive_fwd( *m_graph_fwd,head_backward_copy->state());
#endif

//                                continue;
                                }
                                }
#ifdef DEGUB
                                if (m_problem_bwd.task().direction() =="f"&&m_problem_bwd.task().intersection() =="head")
                                if (head_backward==NULL && head_backward_copy!=NULL && head_backward_copy->state()!=NULL && head_forward!=NULL && head_forward->state()!=NULL){
                                    if((*(head_forward->state())).entails(head_backward_copy->state()->fluent_vec())) {
                                        close_fwd(head_forward);
                                        close_bwd(head_backward_copy);
                                        close_fwd_pri(head_forward);
                                        close_bwd_pri(head_backward_copy);
                                        return {head_backward_copy,head_forward};
                                    }

                                }
#endif
                                continue;
                            }

                            //Generate state
                            if( ! head_backward->has_state() )
                                head_backward->set_state( m_problem_bwd.next(*(head_backward->parent()->state()), head_backward->action()) );
                            if (m_problem_bwd.task().direction() =="e"&&m_problem_bwd.task().intersection() =="head") {
                                if (head_backward != NULL && head_backward->state() != NULL && head_forward != NULL &&
                                    head_forward->state() != NULL) {
                                    if ((*(head_forward->state())).entails(head_backward->state()->fluent_vec())) {
                                        close_fwd(head_forward);
                                        close_bwd(head_backward);
                                        close_fwd_pri(head_forward);
                                        close_bwd_pri(head_backward);
                                        set_max_depth_fwd(head_forward->gn());
                                        return {head_backward, head_forward};
                                    }

                                }
                            }
                            if (m_problem_bwd.task().direction() =="f"&&m_problem_bwd.task().intersection() =="head") {
                                if (head_backward != NULL && head_backward->state() != NULL && head_forward == NULL &&
                                    head_forward_copy != NULL && head_forward_copy->state() != NULL) {
                                    if ((*(head_forward_copy->state())).entails(head_backward->state()->fluent_vec())) {
                                        close_fwd(head_forward_copy);
                                        close_bwd(head_backward);
                                        close_fwd_pri(head_forward_copy);
                                        close_bwd_pri(head_backward);
                                        return {head_backward, head_forward_copy};
                                    }

                                }
                            }
                            if (m_problem_bwd.task().intersection() =="close") {
                                if (head_backward != NULL && head_backward->state() != NULL) {
                                    Fluent_Vec &head_backward_vec = head_backward->state()->fluent_vec();
                                    sort(head_backward_vec.begin(), head_backward_vec.end());


                                    for (auto &I:m_closed_fwd) {
#ifdef DEBUG
                                        // std::cout<<"123"<<std::endl;
                                        // State* tmp=I.second->m_state;
#endif
                                        std::sort(I.second->state()->fluent_vec().begin(),
                                                  I.second->state()->fluent_vec().end());

                                        int left, right;
                                        // auto k1 =  lower_bound(head_backward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                                        auto k1 = lower_bound(I.second->state()->fluent_vec().begin(),
                                                              I.second->state()->fluent_vec().end(),
                                                              head_backward_vec.front());

                                        if (k1 == I.second->state()->fluent_vec().end() ||
                                            (k1 != I.second->state()->fluent_vec().end() &&
                                             (*k1) != head_backward_vec.front()))
                                            continue;
                                        left = k1 - I.second->state()->fluent_vec().begin();


                                        k1 = lower_bound(I.second->state()->fluent_vec().begin(),
                                                         I.second->state()->fluent_vec().end(),
                                                         head_backward_vec.back());

                                        if (k1 == I.second->state()->fluent_vec().end() ||
                                            (k1 != I.second->state()->fluent_vec().end() &&
                                             (*k1) != head_backward_vec.back()))
                                            continue;
                                        right = k1 - I.second->state()->fluent_vec().begin();

                                        int j = 0;
                                        auto &vec = (I.second->state()->fluent_vec());

                                        int i;
                                        for (i = left; i <= right;) {
                                            if (vec[i] == head_backward_vec[j]) {
                                                ++i;
                                                ++j;
                                            } else if (vec[i] < head_backward_vec[j]) ++i;
                                            else if (vec[i] > head_backward_vec[j]) break;
                                        }


                                        if (j == head_backward_vec.size()) {
                                            return {head_backward, I.second};
                                        }
                                    }
                                }
                            }
                          if (m_problem_bwd.task().intersection() =="random") {
                              if (head_backward != NULL && head_backward->state() != NULL) {
                                  Fluent_Vec &head_backward_vec = head_backward->state()->fluent_vec();
                                  sort(head_backward_vec.begin(), head_backward_vec.end());

                                  int _len = m_closed_fwd1.size();


                                  srand(unsigned(time(NULL)));
                                  std::vector<Search_Node *> _q;

                                  for (auto it = m_closed_fwd.begin(); it != m_closed_fwd.end(); ++it) {
                                      (_q).push_back((*it).second);
                                      // (_q)[ans++] = it;
                                  }
                                  // Closed_List_Type _m_close_fwd_copy ;

                                  std::vector<Search_Node *> _m_close_fwd_copy;
                                  for (int i = 1; i <= _len; ++i) {
                                      int id = rand() % m_closed_fwd.size();
                                      _m_close_fwd_copy.push_back(_q[id]);
                                  }

                                  // _q.swap(std::vector<Search_Node*>());

                                  for (auto &(_i):_m_close_fwd_copy) {     // auto &I:m_closed_fwd1

                                      // std::cout<<"123"<<std::endl;
                                      // State* tmp=I.second->m_state;
                                      Search_Node *I = (_i);
                                      std::sort(I->state()->fluent_vec().begin(), I->state()->fluent_vec().end());

                                      int left, right;
                                      // auto k1 =  lower_bound(head_backward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                                      auto k1 = lower_bound(I->state()->fluent_vec().begin(),
                                                            I->state()->fluent_vec().end(), head_backward_vec.front());

                                      if (k1 == I->state()->fluent_vec().end() ||
                                          (k1 != I->state()->fluent_vec().end() && (*k1) != head_backward_vec.front()))
                                          continue;
                                      left = k1 - I->state()->fluent_vec().begin();


                                      k1 = lower_bound(I->state()->fluent_vec().begin(), I->state()->fluent_vec().end(),
                                                       head_backward_vec.back());

                                      if (k1 == I->state()->fluent_vec().end() ||
                                          (k1 != I->state()->fluent_vec().end() && (*k1) != head_backward_vec.back()))
                                          continue;
                                      right = k1 - I->state()->fluent_vec().begin();

                                      int j = 0;
                                      auto &vec = (I->state()->fluent_vec());

                                      int i;
                                      for (i = left; i <= right;) {
                                          if (vec[i] == head_backward_vec[j]) {
                                              ++i;
                                              ++j;
                                          } else if (vec[i] < head_backward_vec[j]) ++i;
                                          else if (vec[i] > head_backward_vec[j]) break;
                                      }


                                      if (j == head_backward_vec.size()) {
                                          return {head_backward, I};    //return {head_backward,I.second};
                                      }
                                  }
                              }
                          }

                          if (m_problem_bwd.task().intersection() =="novelty") {
                              if (head_backward != NULL && head_backward->state() != NULL) {
                                  Fluent_Vec &head_backward_vec = head_backward->state()->fluent_vec();
                                  sort(head_backward_vec.begin(), head_backward_vec.end());

//                              for(auto &I:_m_close_fwd_copy){
                                  for (auto &I:m_closed_fwd1) {

                                      std::sort(I.second->state()->fluent_vec().begin(),
                                                I.second->state()->fluent_vec().end());

                                      int left, right;
                                      // auto k1 =  lower_bound(head_backward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                                      auto k1 = lower_bound(I.second->state()->fluent_vec().begin(),
                                                            I.second->state()->fluent_vec().end(),
                                                            head_backward_vec.front());

                                      if (k1 == I.second->state()->fluent_vec().end() ||
                                          (k1 != I.second->state()->fluent_vec().end() &&
                                           (*k1) != head_backward_vec.front()))
                                          continue;
                                      left = k1 - I.second->state()->fluent_vec().begin();


                                      k1 = lower_bound(I.second->state()->fluent_vec().begin(),
                                                       I.second->state()->fluent_vec().end(), head_backward_vec.back());

                                      if (k1 == I.second->state()->fluent_vec().end() ||
                                          (k1 != I.second->state()->fluent_vec().end() &&
                                           (*k1) != head_backward_vec.back()))
                                          continue;
                                      right = k1 - I.second->state()->fluent_vec().begin();

                                      int j = 0;
                                      auto &vec = (I.second->state()->fluent_vec());

                                      int i;
                                      for (i = left; i <= right;) {
                                          if (vec[i] == head_backward_vec[j]) {
                                              ++i;
                                              ++j;
                                          } else if (vec[i] < head_backward_vec[j]) ++i;
                                          else if (vec[i] > head_backward_vec[j]) break;
                                      }


                                      if (j == head_backward_vec.size()) {
                                          return {head_backward, I.second};
                                      }
                                  }
                              }
                          }
                            if(m_problem_bwd.goal(*(head_backward->state()))) {
                                close_bwd(head_backward);
                                close_bwd_pri(head_backward);
                                set_max_depth_bwd( head_backward->gn() );
                                return {head_backward,NULL};
                            }
                            if ( (time_used() - m_t0_bwd ) > m_time_budget_bwd )
                                return {NULL,NULL};

                            if ( is_closed_bwd( head_backward ) ) {
#ifdef DEBUG
                                if ( m_verbose )
					std::cout << "Already in CLOSED" << std::endl;
#endif
                                delete head_backward;
                                head_backward = get_node_bwd();
                                if (m_problem_bwd.task().direction() =="f") {
                                    if (head_backward != nullptr && head_forward_copy != nullptr) {
                                        head_backward->set_forward_head(head_forward_copy);
                                    }
                                    if (head_backward == NULL && head_forward != NULL) {
                                head_backward_copy= nullptr;
                                head_forward->set_backward_head(nullptr);
#ifdef DEBUG
                                    head_backward_copy= get_node_bwd_pri();
#endif
                                    head_forward->set_backward_head(head_backward_copy);

                                auto m_graph_fwd = this->m_lgm->graph_fwd();

                                graph->compute_lm_graph_set_additive_fwd( *m_graph_fwd);
#ifdef DEBUG
                                    auto m_graph_fwd = this->m_lgm->graph_fwd();

                                    graph->compute_lm_graph_set_additive_fwd( *m_graph_fwd,head_backward_copy->state());
#endif
//                                continue;
                                    }
                                }
#ifdef DEBUG
                                if (m_problem_bwd.task().direction() =="f"&&m_problem_bwd.task().intersection() =="head") {
                                    if (head_backward == NULL && head_backward_copy != NULL &&
                                        head_backward_copy->state() != NULL && head_forward != NULL &&
                                        head_forward->state() != NULL) {
                                        if ((*(head_forward->state())).entails(
                                                head_backward_copy->state()->fluent_vec())) {
                                            close_fwd(head_forward);
                                            close_bwd(head_backward_copy);
                                            close_fwd_pri(head_forward);
                                            close_bwd_pri(head_backward_copy);
                                            return {head_backward_copy, head_forward};
                                        }

                                    }
                                }
#endif

                                continue;
                            }
                            process_bwd(head_backward);
                            close_bwd(head_backward);
                            close_bwd1(head_backward);
                            close_bwd_pri(head_backward);
                            head_backward = get_node_bwd();
#ifdef DEBUG
                            std::cout<<"closed_bwd_1: "<<m_closed_bwd1.size()<<std::endl;
                            std::cout<<"closed_bwd: "<<m_closed_bwd.size()<<std::endl;
#endif
                            if (m_problem_bwd.task().direction() =="f") {
                                if (head_backward == NULL && head_forward != NULL) {
                                    head_backward_copy = nullptr;
                                    head_forward->set_backward_head(nullptr);
#ifdef DEBUG
                                    head_backward_copy = get_node_bwd_pri();
#endif
                                    head_forward->set_backward_head(head_backward_copy);

                                    auto m_graph_fwd = this->m_lgm->graph_fwd();

                                    graph->compute_lm_graph_set_additive_fwd(*m_graph_fwd);
#ifdef DEBUG
                                    auto m_graph_fwd = this->m_lgm->graph_fwd();

                                    graph->compute_lm_graph_set_additive_fwd(*m_graph_fwd, head_backward_copy->state());
#endif

                                    continue;
                                }


# ifdef DEBUG
                                //                            if (head_backward==NULL && head_backward_copy!=NULL && head_backward_copy->state()!=NULL && head_forward!=NULL && head_forward->state()!=NULL){
                                //                                if((*(head_forward->state())).entails(head_backward_copy->state()->fluent_vec())) {
                                //                                    close_fwd(head_forward);
                                //                                    close_bwd(head_backward_copy);
                                //                                    close_fwd_pri(head_forward);
                                //                                    close_bwd_pri(head_backward_copy);
                                //                                    return {head_backward_copy,head_forward};
                                //                                }
                                //
                                //                            }
#endif
                                if (head_backward != NULL && !head_backward->has_state())
                                    head_backward->set_state(m_problem_bwd.next(*(head_backward->parent()->state()),
                                                                                head_backward->action()));
# ifdef DEBUG
                                //                            if (head_backward!=NULL && head_backward->state()!=NULL){
                                //                                Fluent_Vec &head_backward_vec=head_backward->state()->fluent_vec();
                                //                                sort(head_backward_vec.begin(),head_backward_vec.end());
                                //
                                //
                                //                                for(auto &I:m_closed_fwd){
                                //
                                //                                    // std::cout<<"123"<<std::endl;
                                //                                    // State* tmp=I.second->m_state;
                                //                                    std::sort(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end());
                                //
                                //                                    int left,right;
                                //                                    // auto k1 =  lower_bound(head_backward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                                //                                    auto k1 =  lower_bound(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end(),head_backward_vec.front()) ;
                                //
                                //                                    if(k1==I.second->state()->fluent_vec().end() || ( k1!=I.second->state()->fluent_vec().end()  &&(*k1)!=head_backward_vec.front())) continue;
                                //                                    left = k1 - I.second->state()->fluent_vec().begin();
                                //
                                //
                                //                                    k1 = lower_bound(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end(),head_backward_vec.back()) ;
                                //
                                //                                    if(k1==I.second->state()->fluent_vec().end() || (  k1!=I.second->state()->fluent_vec().end() && (*k1)!=head_backward_vec.back())) continue;
                                //                                    right = k1 - I.second->state()->fluent_vec().begin();
                                //
                                //                                    int j = 0;
                                //                                    auto &vec = (I.second->state()->fluent_vec());
                                //
                                //                                    int i;
                                //                                    for(i=left;i<=right;){
                                //                                        if(vec[i]==head_backward_vec[j]){
                                //                                            ++i;
                                //                                            ++j;
                                //                                        }else if(vec[i]<head_backward_vec[j]) ++i;
                                //                                        else if(vec[i]>head_backward_vec[j]) break;
                                //                                    }
                                //
                                //
                                //                                    if(j==head_backward_vec.size()) {
                                //                                        return {head_backward,I.second};
                                //                                    }
                                //                                }
                                //                            }
                                                                //check_goal_in_closed_fwd(head_backward);
                                //
                                //                            if (head_backward!=NULL && head_backward->state()!=NULL){
                                //                                Fluent_Vec &head_backward_vec=head_backward->state()->fluent_vec();
                                //                                sort(head_backward_vec.begin(),head_backward_vec.end());
                                //
                                //
                                //                                for(auto &I:m_closed_fwd1){
                                //
                                //                                    // std::cout<<"123"<<std::endl;
                                //                                    // State* tmp=I.second->m_state;
                                //                                    std::sort(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end());
                                //
                                //                                    int left,right;
                                //                                    // auto k1 =  lower_bound(head_backward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                                //                                    auto k1 =  lower_bound(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end(),head_backward_vec.front()) ;
                                //
                                //                                    if(k1==I.second->state()->fluent_vec().end() || ( k1!=I.second->state()->fluent_vec().end()  &&(*k1)!=head_backward_vec.front())) continue;
                                //                                    left = k1 - I.second->state()->fluent_vec().begin();
                                //
                                //
                                //                                    k1 = lower_bound(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end(),head_backward_vec.back()) ;
                                //
                                //                                    if(k1==I.second->state()->fluent_vec().end() || (  k1!=I.second->state()->fluent_vec().end() && (*k1)!=head_backward_vec.back())) continue;
                                //                                    right = k1 - I.second->state()->fluent_vec().begin();
                                //
                                //                                    int j = 0;
                                //                                    auto &vec = (I.second->state()->fluent_vec());
                                //
                                //                                    int i;
                                //                                    for(i=left;i<=right;){
                                //                                        if(vec[i]==head_backward_vec[j]){
                                //                                            ++i;
                                //                                            ++j;
                                //                                        }else if(vec[i]<head_backward_vec[j]) ++i;
                                //                                        else if(vec[i]>head_backward_vec[j]) break;
                                //                                    }
                                //
                                //
                                //                                    if(j==head_backward_vec.size()) {
                                //                                        return {head_backward,I.second};
                                //                                    }
                                //                                }
                                //                            }
                                //                            if (head_backward!=NULL && head_backward->state()!=NULL && head_forward!=NULL && head_forward->state()!=NULL){
                                //                                if((*(head_forward->state())).entails(head_backward->state()->fluent_vec())) {
                                //                                    close_fwd(head_forward);
                                //                                    close_bwd(head_backward);
                                //                                    close_fwd_pri(head_forward);
                                //                                    close_bwd_pri(head_backward);
                                //                                    set_max_depth_fwd( head_forward->gn() );
                                //                                    return {head_backward,head_forward};
                                //                                }
                                //
                                //                            }
#endif
                                if (head_backward != nullptr && head_forward != nullptr) {
                                    head_backward_copy = new Search_Node(*head_backward);
                                    head_forward->set_backward_head(head_backward_copy);

                                    auto m_graph_fwd = this->m_lgm->graph_fwd();

                                    graph->compute_lm_graph_set_additive_fwd(*m_graph_fwd, head_backward->state());

                                }
                            }
#ifdef DEBUG
//                            if (head_backward!=NULL && head_backward->state()!=NULL && head_forward==NULL && head_forward_copy!=NULL && head_forward_copy->state()!=NULL){
//                                if((*(head_forward_copy->state())).entails(head_backward->state()->fluent_vec())) {
//                                    close_fwd(head_forward_copy);
//                                    close_bwd(head_backward);
//                                    close_fwd_pri(head_forward_copy);
//                                    close_bwd_pri(head_backward);
//                                    return {head_backward,head_forward_copy};
//                                }
//
//                            }
#endif
                            break;


                        }

#if 0
//                        if (head_backward!=NULL && head_forward!=NULL){
//                            if((*(head_forward->state())).entails(head_backward->state()->fluent_vec())) {
//                                close_fwd(head_forward);
//                                close_bwd(head_backward);
//                                set_max_depth_fwd( head_forward->gn() );
//                                return {head_backward,head_forward};
//                            }
//                        }
#endif

                   }


                    return {NULL,NULL};



                }

                virtual std::pair<Search_Node*,Search_Node*> check_goal_in_closed_list(  Search_Node *head_forward,Search_Node *head_backward) {
                    Fluent_Vec &head_backward_vec = head_backward->state()->fluent_vec();
                    sort(head_backward_vec.begin(), head_backward_vec.end());


                    for (auto &I:m_closed_fwd) {

                        // std::cout<<"123"<<std::endl;
                        // State* tmp=I.second->m_state;
                        std::sort(I.second->state()->fluent_vec().begin(), I.second->state()->fluent_vec().end());

                        int left, right;
                        // auto k1 =  lower_bound(head_backward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                        auto k1 = lower_bound(I.second->state()->fluent_vec().begin(),
                                              I.second->state()->fluent_vec().end(), head_backward_vec.front());

                        if (k1 == I.second->state()->fluent_vec().end() ||
                            (k1 != I.second->state()->fluent_vec().end() && (*k1) != head_backward_vec.front()))
                            continue;
                        left = k1 - I.second->state()->fluent_vec().begin();


                        k1 = lower_bound(I.second->state()->fluent_vec().begin(), I.second->state()->fluent_vec().end(),
                                         head_backward_vec.back());

                        if (k1 == I.second->state()->fluent_vec().end() ||
                            (k1 != I.second->state()->fluent_vec().end() && (*k1) != head_backward_vec.back()))
                            continue;
                        right = k1 - I.second->state()->fluent_vec().begin();

                        int j = 0;
                        auto &vec = (I.second->state()->fluent_vec());

                        int i;
                        for (i = left; i <= right;) {
                            if (vec[i] == head_backward_vec[j]) {
                                ++i;
                                ++j;
                            } else if (vec[i] < head_backward_vec[j]) ++i;
                            else if (vec[i] > head_backward_vec[j]) break;
                        }


                        if (j == head_backward_vec.size()) {
                            return {head_backward, I.second};
                            break;
                        }
                    }

                    Fluent_Vec &head_forward_vec = head_forward->state()->fluent_vec();
                    sort(head_forward_vec.begin(), head_forward_vec.end());

                    //    std::vector<unsigned int>* idx =NULL;
                    for (auto &I:m_closed_bwd) {

                        // std::cout<<"123"<<std::endl;
                        // State* tmp=I.second->m_state;
                        std::sort(I.second->state()->fluent_vec().begin(), I.second->state()->fluent_vec().end());

                        int left, right;
                        auto k1 = lower_bound(head_forward_vec.begin(), head_forward_vec.end(),
                                              I.second->state()->fluent_vec().front());
                        if (k1 == head_forward_vec.end() ||
                            (k1 != head_forward_vec.end() && (*k1) != I.second->state()->fluent_vec().front()))
                            continue;
                        left = k1 - head_forward_vec.begin();
                        k1 = lower_bound(head_forward_vec.begin(), head_forward_vec.end(),
                                         I.second->state()->fluent_vec().back());

                        if (k1 == head_forward_vec.end() ||
                            (k1 != head_forward_vec.end() && (*k1) != I.second->state()->fluent_vec().back()))
                            continue;
                        right = k1 - head_forward_vec.begin();

                        int j = 0;
                        auto &vec = (I.second->state()->fluent_vec());
                        int i;
                        for (i = left; i <= right;) {
                            if (vec[j] == head_forward_vec[i]) {
                                ++i;
                                ++j;
                            } else if (vec[j] < head_forward_vec[i]) break;
                            else if (vec[j] > head_forward_vec[i]) ++i;
                        }


                        if (j == vec.size()) {
                            return {I.second, head_forward};
                            break;
                        }
                    }
                }


                virtual std::pair<Search_Node*,Search_Node*> check_goal_in_closed_bwd(  Search_Node *head_forward){

                    Fluent_Vec &head_forward_vec=head_forward->state()->fluent_vec();
                    sort(head_forward_vec.begin(),head_forward_vec.end());

                    for(auto &I:m_closed_bwd){

                        std::sort(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end());

                        int left,right;
                        auto k1 =  lower_bound(head_forward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                        if(k1==head_forward_vec.end() || ( k1!=head_forward_vec.end()  &&(*k1)!=I.second->state()->fluent_vec().front())) continue;
                        left = k1 - head_forward_vec.begin();
                        k1 = lower_bound(head_forward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().back()) ;

                        if(k1==head_forward_vec.end() || (  k1!=head_forward_vec.end() && (*k1)!=I.second->state()->fluent_vec().back())) continue;
                        right = k1 - head_forward_vec.begin();

                        int j = 0;
                        auto &vec = (I.second->state()->fluent_vec());
                        int i;
                        for(i=left;i<=right;){
                            if(vec[j]==head_forward_vec[i]){
                                ++i;
                                ++j;
                            }else if(vec[j]<head_forward_vec[i]) break;
                            else if(vec[j]>head_forward_vec[i]) ++i;
                        }


                        if(j==vec.size()) {
                            return {I.second,head_forward};
                        }
                    }
                }

                virtual std::pair<Search_Node*,Search_Node*> check_goal_in_closed_fwd(  Search_Node *head_backward){

                    Fluent_Vec &head_backward_vec=head_backward->state()->fluent_vec();
                    sort(head_backward_vec.begin(),head_backward_vec.end());


                    for(auto &I:m_closed_fwd){

                        // std::cout<<"123"<<std::endl;
                        // State* tmp=I.second->m_state;
                        std::sort(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end());

                        int left,right;
                        // auto k1 =  lower_bound(head_backward_vec.begin(),head_forward_vec.end(),I.second->state()->fluent_vec().front()) ;
                        auto k1 =  lower_bound(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end(),head_backward_vec.front()) ;

                        if(k1==I.second->state()->fluent_vec().end() || ( k1!=I.second->state()->fluent_vec().end()  &&(*k1)!=head_backward_vec.front())) continue;
                        left = k1 - I.second->state()->fluent_vec().begin();


                        k1 = lower_bound(I.second->state()->fluent_vec().begin(),I.second->state()->fluent_vec().end(),head_backward_vec.back()) ;

                        if(k1==I.second->state()->fluent_vec().end() || (  k1!=I.second->state()->fluent_vec().end() && (*k1)!=head_backward_vec.back())) continue;
                        right = k1 - I.second->state()->fluent_vec().begin();

                        int j = 0;
                        auto &vec = (I.second->state()->fluent_vec());

                        int i;
                        for(i=left;i<=right;){
                            if(vec[i]==head_backward_vec[j]){
                                ++i;
                                ++j;
                            }else if(vec[i]<head_backward_vec[j]) ++i;
                            else if(vec[i]>head_backward_vec[j]) break;
                        }


                        if(j==head_backward_vec.size()) {
                            return {head_backward,I.second};
                        }
                    }

                }

                virtual std::pair<Search_Node*,Search_Node*> do_search_bwd(){

                    Search_Node *head_backward = get_node_bwd();





//
//                    while (head_forward){


                    while(head_backward) {
                        if (head_backward==NULL){
                            break;
                        }





                        if (head_backward!=NULL){
                            if ( head_backward->gn() >= max_depth_bwd() )  {
                                close_bwd(head_backward);
                                head_backward = get_node_bwd();


                                continue;
                            }

                            //Generate state
                            if( ! head_backward->has_state() )
                                head_backward->set_state( m_problem_bwd.next(*(head_backward->parent()->state()), head_backward->action()) );

                            if(m_problem_bwd.goal(*(head_backward->state()))) {
                                close_bwd(head_backward);
                                set_max_depth_bwd( head_backward->gn() );
                                return {head_backward,NULL};
                            }
                            if ( (time_used() - m_t0_bwd ) > m_time_budget_bwd )
                                return {NULL,NULL};

                            if ( is_closed_bwd( head_backward ) ) {
#ifdef DEBUG
                                if ( m_verbose )
					std::cout << "Already in CLOSED" << std::endl;
#endif
                                delete head_backward;
                                head_backward = get_node_bwd();



                                continue;
                            }
                            process_bwd(head_backward);
                            close_bwd(head_backward);
                            head_backward = get_node_bwd();
                        }
                        //Generate state
                        if( head_backward!=NULL && ! head_backward->has_state() )
                            head_backward->set_state( m_problem_bwd.next(*(head_backward->parent()->state()), head_backward->action()) );




                    }


                    return {NULL,NULL};



                }
                virtual std::pair<Search_Node*,Search_Node*> do_search_fwd(){


                    Search_Node  *head_forward =  get_node_fwd() ;


                    while(head_forward ) {


                        if (head_forward!=NULL){
                            if ( head_forward->gn() >= max_depth_fwd() )  {
                                close_fwd(head_forward);
                                head_forward= get_node_fwd();


                                continue;
                            }

                            //Generate state
                            if( ! head_forward->has_state() )
                                head_forward->set_state( m_problem_fwd.next(*(head_forward->parent()->state()), head_forward->action()) );

                            if(m_problem_fwd.goal(*(head_forward->state()))) {
                                close_fwd(head_forward);
                                set_max_depth_fwd( head_forward->gn() );
                                return {NULL,head_forward};
                            }
                            if ( (time_used() - m_t0_fwd ) > m_time_budget_fwd )
                                return {NULL,NULL};

                            if ( is_closed_fwd( head_forward ) ) {
#ifdef DEBUG
                                if ( m_verbose )
					std::cout << "Already in CLOSED" << std::endl;
#endif
                                delete head_forward;
                                head_forward = get_node_fwd();


                                continue;
                            }
                            process_fwd(head_forward);
                            close_fwd(head_forward);
                            close_fwd1(head_forward);
#ifdef DEBUG
                            std::cout<<"closed_fwd_1: "<<m_closed_fwd1.size()<<std::endl;
                            std::cout<<"closed_fwd: "<<m_closed_fwd.size()<<std::endl;
#endif

                            head_forward = get_node_fwd();

                        }

                        if( head_forward!=NULL && ! head_forward->has_state() )
                            head_forward->set_state( m_problem_fwd.next(*(head_forward->parent()->state()), head_forward->action()) );


                    }


                    return {NULL,NULL};



                }



                /** check later
                 *
                 * @param v
                 * @param g
                 */
                void	set_arity_bwd( float v, unsigned g ){ m_first_h->set_arity_bwd( v, g ); }

                void    set_max_novelty_bwd( unsigned v )   { m_max_novelty_bwd = v; }

                void	set_arity_fwd( float v, unsigned g ){ m_first_h->set_arity_fwd( v, g ); }
                void    set_max_novelty_fwd( unsigned v )   { m_max_novelty_fwd = v; }


                bool	find_solution( float& cost_bwd, float& cost_fwd,std::vector<Action_Idx>& plan, Gen_Lms* graph=NULL) {
                    m_t0_bwd = time_used();
                    m_t0_fwd = time_used();
                    std::pair<Search_Node*,Search_Node*> end= do_search(graph);
                    if ( end.first==NULL && end.second==NULL ) return false;
                     if (end.first!=NULL&& end.second==NULL)
                     {
                         std::cout<<"Backward Search Found Plan"<<std::endl;
                         extract_plan_bwd( m_root_bwd, end.first, plan, cost_bwd );
                     }

                     else if(end.second!=NULL&& end.first==NULL)
                     {
                         std::cout<<"Forward Search Found Plan"<<std::endl;
                         extract_plan_fwd( m_root_fwd, end.second, plan, cost_fwd );
                     }

                    else{
                         std::cout <<"Found Plan With Joined Search"<<std::endl;
                         extract_plan_bwd_fwd( m_root_bwd, end.first,m_root_fwd,end.second, plan, cost_bwd,cost_fwd );
                    }


                    return true;
                }
                bool	find_solution_forward( float& cost_fwd,std::vector<Action_Idx>& plan, Gen_Lms* graph=NULL) {
                    m_t0_fwd = time_used();
                    std::pair<Search_Node*,Search_Node*> end= do_search_fwd();
                    if ( end.second==NULL )
                        {return false;
                        } else
                    {
                        std::cout<<"Forward Search Found Plan"<<std::endl;
                        extract_plan_fwd( m_root_fwd, end.second, plan, cost_fwd );
                    }


                    //Search_Node* s_bwd, Search_Node* t_bwd,Search_Node* s_fwd,Search_Node* t_fwd, std::vector<Action_Idx>& plan, float& cost

                    return true;
                }

                bool	find_solution_backward( float& cost_bwd,std::vector<Action_Idx>& plan, Gen_Lms* graph=NULL) {
                    m_t0_bwd = time_used();
                    std::pair<Search_Node*,Search_Node*> end= do_search_bwd();
                    /** need to added
                     *
                     */
                    if (end.first==NULL)
                    {
                        return false;
                    }

                    else
                    {
                        std::cout <<"Backward Search Found Planh"<<std::endl;
                        extract_plan_bwd( m_root_bwd, end.first, plan, cost_bwd );
                    }

                    //Search_Node* s_bwd, Search_Node* t_bwd,Search_Node* s_fwd,Search_Node* t_fwd, std::vector<Action_Idx>& plan, float& cost

                    return true;
                }

                float			max_depth_bwd() const			{ return m_max_depth_bwd; }
                void			set_max_depth_bwd( float v ) 		{ m_max_depth_bwd = v; }
                void			set_use_rp_bwd( bool v )     	 { m_use_rp_bwd= v; }
                void			set_use_rp_from_init_only_bwd( bool v ) { m_use_rp_from_init_only_bwd = v; }
                void			set_use_novelty_bwd( bool v ) 	 { m_use_novelty_bwd = v; }
                void			set_use_novelty_pruning_bwd( bool v ){ m_use_novelty_pruning_bwd = v; }

                void			inc_gen_bwd()			{ m_gen_count_bwd++; }
                unsigned		generated_bwd() const		{ return m_gen_count_bwd; }
                void			inc_eval_bwd()			{ m_exp_count_bwd++; }
                unsigned		expanded_bwd() const		{ return m_exp_count_bwd; }
                void			inc_dead_end_bwd()			{ m_dead_end_count_bwd++; }
                unsigned		dead_ends_bwd() const		{ return m_dead_end_count_bwd; }
                void			inc_replaced_open_bwd()		{ m_open_repl_count_bwd++; }
                unsigned		open_repl_bwd() const		{ return m_open_repl_count_bwd; }

                void			set_budget_bwd( float v ) 		{ m_time_budget_bwd = v; }
                float			time_budget_bwd() const		{ return m_time_budget_bwd; }

                float			t0_bwd() const			{ return m_t0_bwd; }

                void 			close_bwd( Search_Node* n ) 	{  m_closed_bwd.put(n); }
                Closed_List_Type&	closed_bwd() 			{ return m_closed_bwd; }

                const	bwd_Search_Problem&	problem_backward() const			{ return m_problem_bwd; }
                const	Fwd_Search_Problem&	problem_forward() const			{ return m_problem_fwd; }


                float			max_depth_fwd() const			{ return m_max_depth_fwd; }
                void			set_max_depth_fwd( float v ) 		{ m_max_depth_fwd = v; }
                void			set_use_rp_fwd( bool v )     	 { m_use_rp_fwd = v; }
                void			set_use_rp_from_init_only_fwd( bool v ) { m_use_rp_from_init_only_fwd = v; }
                void			set_use_novelty_fwd( bool v ) 	 { m_use_novelty_fwd = v; }
                void			set_use_novelty_pruning_fwd( bool v ){ m_use_novelty_pruning_fwd = v; }

                void			inc_gen_fwd()			{ m_gen_count_fwd++; }
                unsigned		generated_fwd() const		{ return m_gen_count_fwd; }
                void			inc_eval_fwd()			{ m_exp_count_fwd++; }
                unsigned		expanded_fwd() const		{ return m_exp_count_fwd; }
                void			inc_dead_end_fwd()			{ m_dead_end_count_fwd++; }
                unsigned		dead_ends_fwd() const		{ return m_dead_end_count_fwd; }
                void			inc_replaced_open_fwd()		{ m_open_repl_count_fwd++; }
                unsigned		open_repl_fwd() const		{ return m_open_repl_count_fwd; }

                void			set_budget_fwd( float v ) 		{ m_time_budget_fwd = v; }
                float			time_budget_fwd() const		{ return m_time_budget_fwd; }

                float			t0_fwd() const			{ return m_t0_fwd; }

                void 			close_fwd( Search_Node* n ) 	{  m_closed_fwd.put(n); }


                bool _check(Search_Node* n){
                    if (n->m_novelty==1) return true;
                    return  false;

                }
                /*
                 * check later
                 */
                void 			close_fwd_pri( Search_Node* n ) 	{  m_closed_fwd_pri.insert_close(n); }
                void 			close_bwd_pri( Search_Node* n ) 	{  m_closed_bwd_pri.insert_close(n); }
                void 			close_fwd1( Search_Node* n ) 	{  if (_check(n)) m_closed_fwd1.put(n); }
                void 			close_bwd1( Search_Node* n ) 	{  if (_check(n)) m_closed_bwd1.put(n); }

                Closed_List_Type&	closed_fwd() 			{ return m_closed_fwd; }

                Closed_List_Type&	closed_fwd_pri() 			{ return m_closed_fwd_pri; }
                Closed_List_Type&	closed_bwd_pri() 			{ return m_closed_bwd_pri; }







                First_Heuristic&	h1()				{ return *m_first_h; }
                Second_Heuristic&	h2()				{ return *m_second_h; }
                Relevant_Fluents_Heuristic&	rel_fl_h()	        { return *m_relevant_fluents_h; }

                void			set_verbose_bwd( bool v ) 		{ m_verbose_bwd  = v; }
                void			set_verbose_fwd( bool v ) 		{ m_verbose_fwd  = v; }
                /** check later
                 *
                 * @param lgm
                 */
                void                    use_land_graph_manager( Landmarks_Graph_Manager* lgm ) {
                    m_lgm= lgm;
                    m_second_h->set_graph_bwd( m_lgm->graph_bwd() );
                    m_second_h->set_graph_fwd( m_lgm->graph_fwd() );
                }
#if 0
//                void                    use_land_graph_manager_fwd( Landmarks_Graph_Manager* lgm ) {
//                    m_lgm_fwd = lgm;
//                    m_second_h->set_graph_fwd( m_lgm_fwd->graph_fwd() );
//                }
#endif

            protected:

                virtual void	extract_plan_bwd( Search_Node* s, Search_Node* t, std::vector<Action_Idx>& plan, float& cost ) {
                    Search_Node *tmp = t;
                    cost = 0.0f;
                    while( tmp != s) {
                        cost += m_problem_bwd.cost( *(tmp->state()), tmp->action() );
                        plan.push_back(tmp->action());
                        tmp = tmp->parent();
                    }


                }
                virtual void	extract_plan_fwd( Search_Node* s, Search_Node* t, std::vector<Action_Idx>& plan, float& cost ) {
                    Search_Node *tmp = t;
                    cost = 0.0f;
                    while( tmp != s) {
                        cost += m_problem_fwd.cost( *(tmp->state()), tmp->action() );
                        plan.push_back(tmp->action());
                        tmp = tmp->parent();
                    }

                    std::reverse(plan.begin(), plan.end());
                }

                virtual void	extract_plan_bwd_fwd( Search_Node* s_bwd, Search_Node* t_bwd,Search_Node* s_fwd,Search_Node* t_fwd, std::vector<Action_Idx>& plan, float& cost_bwd,float&cost_fwd ) {
                    Search_Node *tmp = t_fwd;
                    cost_fwd = 0.0f;
                    while( tmp != s_fwd) {
                        cost_fwd += m_problem_fwd.cost( *(tmp->state()), tmp->action() );
                        plan.push_back(tmp->action());
                        tmp = tmp->parent();
                    }

                    std::reverse(plan.begin(), plan.end());

                    tmp = t_bwd;
                    cost_bwd = 0.0f;
                    while( tmp != s_bwd) {
                        cost_bwd += m_problem_bwd.cost( *(tmp->state()), tmp->action() );
                        plan.push_back(tmp->action());
                        tmp = tmp->parent();
                    }


                }


                void	extract_path( Search_Node* s, Search_Node* t, std::vector<Search_Node*>& plan ) {
                    Search_Node* tmp = t;
                    while( tmp != s) {
                        plan.push_back(tmp);
                        tmp = tmp->parent();
                    }

                    std::reverse(plan.begin(), plan.end());
                }

            protected:

                const bwd_Search_Problem&			m_problem_bwd;
                const Fwd_Search_Problem&			m_problem_fwd;
//                Abstract_Novelty*      			m_novelty;
                First_Heuristic*			m_first_h;
               // First_Heuristic*			m_first_h_forward;
                Second_Heuristic*			m_second_h;

              //  Second_Heuristic*			m_second_h_forward;
                Relevant_Fluents_Heuristic*    		m_relevant_fluents_h;
              //  Relevant_Fluents_Heuristic*    		m_relevant_fluents_h_forward;


                //Open_List_Type				m_open;
                Open_List_Type				m_open_bwd;
                Open_List_Type				m_open_fwd;
//                Open_List_Type				m_open_forward;
//                Open_List_Type				m_open_backward;
               // Closed_List_Type			m_closed;
                Closed_List_Type			m_closed_bwd;
                Closed_List_Type			m_closed_fwd;
                Closed_List_Type			m_closed_bwd1;
                Closed_List_Type			m_closed_fwd1;

                Open_List_Type				m_closed_bwd_pri;
                Open_List_Type				m_closed_fwd_pri;


                unsigned				m_exp_count_bwd;
                unsigned				m_gen_count_bwd;
                unsigned				m_dead_end_count_bwd;
                unsigned				m_open_repl_count_bwd;

                unsigned				m_exp_count_fwd;
                unsigned				m_gen_count_fwd;
                unsigned				m_dead_end_count_fwd;
                unsigned				m_open_repl_count_fwd;

                float					m_max_depth_bwd;
                unsigned                m_max_novelty_bwd;
                float					m_time_budget_bwd;
                float					m_t0_bwd;

                float					m_max_depth_fwd;
                unsigned                m_max_novelty_fwd;
                float					m_time_budget_fwd;
                float					m_t0_fwd;

              //  Search_Node*				m_root;
                Search_Node*				m_root_bwd;
                Search_Node*				m_root_fwd;
//                Search_Node*				m_root_forward;
//                Search_Node*				m_root_backward;
         //       Search_Node*				m_root_edit;

                Search_Node*				m_root_edit_bwd;
                Search_Node*				m_root_edit_fwd;
            //   std::vector<Action_Idx> 		m_app_set;

                std::vector<Action_Idx> 		m_app_set_bwd;
                std::vector<Action_Idx> 		m_app_set_fwd;

                Landmarks_Graph_Manager*                m_lgm;
//                Landmarks_Graph_Manager*                m_lgm_bwd;
//                Landmarks_Graph_Manager*                m_lgm_fwd;

                unsigned                                m_max_h2n_bwd;
                unsigned                                m_max_r_bwd;
                bool					m_verbose_bwd;

                unsigned                                m_max_h2n_fwd;
                unsigned                                m_max_r_fwd;
                bool					m_verbose_fwd;

                bool                                    m_use_novelty_bwd;
                bool                                    m_use_novelty_pruning_bwd;
                bool                                    m_use_rp_bwd;
                bool                                    m_use_rp_from_init_only_bwd;
                std::vector<bool>	 						m_in_negation_bwd;

                bool                                    m_use_novelty_fwd;
                bool                                    m_use_novelty_pruning_fwd;
                bool                                    m_use_rp_fwd;
                bool                                    m_use_rp_from_init_only_fwd;
                std::vector<bool>	 						m_in_negation_fwd;

            };

        }

    }

}


