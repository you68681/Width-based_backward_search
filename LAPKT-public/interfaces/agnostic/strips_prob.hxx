
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

#ifndef __STRIPS_PROBLEM__
#define __STRIPS_PROBLEM__

#include <string>
#include <map>
#include <iosfwd>
#include <types.hxx>
#include <succ_gen.hxx>
#include <match_tree.hxx>
#include <algorithm>
#include <mutex_set.hxx>

namespace aptk
{

	class STRIPS_Problem
	{
	public:
		class Best_Supporter {
		public:
			Best_Supporter() 
				: act_idx( no_such_index ), eff_idx ( no_such_index ) {
			}
	
			Best_Supporter( unsigned a_index, unsigned e_index ) 
				: act_idx( a_index ), eff_idx( e_index ) {
			}
	
			bool operator==( const Best_Supporter& other ) const {
				return act_idx == other.act_idx && eff_idx == other.eff_idx;
			}

			bool operator<( const Best_Supporter& other ) const {
				if( act_idx < other.act_idx ) return true;
				if( act_idx == other.act_idx )
					return eff_idx < other.eff_idx;
				return false;
			}
	
			unsigned act_idx;
			unsigned eff_idx;
		};

		class Trigger {
		public:

			Trigger( unsigned nf, const Fluent_Vec& prec, const Fluent_Vec& eff )
			: m_last( 0 ) {
				m_condition = prec;
				m_effect = eff;
				m_last = m_condition.size()-1;
				m_cond_pending = m_condition.size();
				m_cond_status.resize( nf );
			}

			Trigger( unsigned nf, const Fluent_Vec& prec, const Fluent_Vec& cond, const Fluent_Vec& eff )
			: m_last( 0 ) {
				m_condition = prec;
				for ( auto p : cond )
					if ( std::find( m_condition.begin(), m_condition.end(), p ) == m_condition.end() ) 
						m_condition.push_back( p );
				m_effect = eff;
				m_last = m_condition.size()-1;
				m_cond_pending = m_condition.size();
				m_cond_status.resize( nf );
			}


                        Trigger (const Trigger & other) {
                                m_condition = std::move( other.m_condition );
				m_effect = std::move( other.m_effect );
				m_last = other.m_last;
				m_cond_pending = other.m_cond_pending;
				m_cond_status = std::move( other.m_cond_status );
                        }

			Trigger( Trigger&& other ) {
				m_condition = std::move( other.m_condition );
				m_effect = std::move( other.m_effect );
				m_last = other.m_last;
				m_cond_pending = other.m_cond_pending;
				m_cond_status = std::move( other.m_cond_status );
			}

			~Trigger() {
			}

			void	reset() {
				//m_last = m_condition.size()-1;
				for ( unsigned c : m_condition )
					m_cond_status.set( c );
				m_cond_pending = m_condition.size();
			}

			bool	satisfied() const { 
				//return m_last == -1;
				return m_cond_pending == 0;
			}

			void	update( unsigned p ) {
				/*
				for ( int k = 0; k <= m_last ; k++ ) {
					if ( m_condition[k] == p ) {
						std::swap( m_condition[k], m_condition[m_last] );
						m_last--;
						return;
					} 
				}
				*/
				if ( !m_cond_status.isset(p) ) return;
				m_cond_pending--;
				m_cond_status.unset(p);
			}
			
			const Fluent_Vec& condition() const { return m_condition; }
			const Fluent_Vec& effect() const { return m_effect; }

			Fluent_Vec	m_condition;
			Fluent_Vec	m_effect;
			int		m_last;
			Bit_Array	m_cond_status;
			int		m_cond_pending;

		};

		STRIPS_Problem( std::string dom_name = "Unnamed", std::string prob_name = "Unnamed ");
		virtual ~STRIPS_Problem();

		void			set_domain_name( std::string name ) { m_domain_name = name; }
		void			set_problem_name( std::string name ) { m_problem_name = name; }
		std::string		domain_name() const { return m_domain_name; }
		std::string		problem_name() const { return m_problem_name; }
        /** chao edit
         *
         * @return
         */
        unsigned 		num_negation_fluents() const		{ return m_negation_num_fluents; }
        unsigned 		num_negation_actions() const		{ return m_negation_num_actions; }

        void			set_direction( std::string name ) { m_direction = name; }
        void			set_intersection( std::string name ) { m_intersection = name; }
        std::string		direction() const { return m_direction; }
        std::string		intersection() const { return m_intersection; }

		unsigned 		num_fluents() const		{ return m_num_fluents; }
		unsigned 		num_actions() const		{ return m_num_actions; }

		void			set_num_fluents( unsigned nf ) { m_num_fluents = nf; }
		void			set_num_actions( unsigned na ) { m_num_actions = na; }

		static unsigned 	add_action( STRIPS_Problem& p, std::string signature,
						    const Fluent_Vec& pre, const Fluent_Vec& add, const Fluent_Vec& del,
						    const Conditional_Effect_Vec& ceffs, float cost = 1.0f );

        /** chao edit
         *
         */
        static unsigned 	add_negation_action( STRIPS_Problem& p, std::string signature,
                                       const Fluent_Vec& pre, const Fluent_Vec& add, const Fluent_Vec& del,
                                       const Conditional_Effect_Vec& ceffs, float cost = 1.0f );

		static unsigned 	add_fluent( STRIPS_Problem& p, std::string signature );

        /** chao edit
         *
         * @param p
         * @param signature
         * @return
         */
        static unsigned 	add_fluent_edit( STRIPS_Problem& p, std::string signature, std::vector<unsigned >);
        static unsigned 	add_fluent_negation( STRIPS_Problem& p, std::string signature);

		static void		set_init( STRIPS_Problem& p, const Fluent_Vec& init );
		/** chao add
		 *
		 */
		static void     set_init_negation(STRIPS_Problem& p, const Fluent_Vec& negation);

	    static void		set_goal( STRIPS_Problem& p, const Fluent_Vec& goal, bool createEndOp = false, bool keep_original_goal = false );

		static void		make_delete_relaxation( const STRIPS_Problem& orig, STRIPS_Problem& relaxed );

	  	
		Fluent_Ptr_Vec&		fluents() 			{ return m_fluents; }
		/** chao add
		 *
		 * @return
		 */
//        const std::vector< const Negation_Fluent*>&
//        negation_fluents() const			{ return m_const_negation_fluents; }
//        Negation_Fluent_Ptr_Vec&		negation_fluents() 			{ return m_negation_fluents; }
//
//        Negation_Action_Ptr_Vec&		negation_actions() 			{ return m_negation_actions; }
        const std::vector< const Fluent*>&
        negation_fluents() const			{ return m_const_negation_fluents; }
        Fluent_Ptr_Vec&		negation_fluents() 			{ return m_negation_fluents; }

        Action_Ptr_Vec&		negation_actions() 			{ return m_negation_actions; }

		Action_Ptr_Vec&		actions() 			{ return m_actions; }
		const std::vector< const Fluent*>&	
					fluents() const			{ return m_const_fluents; }
		const std::vector< const Action*>&
					actions() const			{ return m_const_actions; }
		/** chao add
		 *
		 */
//        const std::vector< const Negation_Action*>&
//        negation_actions() const			{ return m_const_negation_actions; }
        const std::vector< const Action*>&
        negation_actions() const			{ return m_const_negation_actions; }
		Fluent_Vec&		init()	  			{ return m_init; }
        Fluent_Vec&		init_negation()	  			{ return m_init_negation; }
		Fluent_Vec&		goal()	  			{ return m_goal; }
		const Fluent_Vec&	init() const  			{ return m_init; }
		const Fluent_Vec&	goal() const  			{ return m_goal;}
		const Fluent_Vec&	init_negation() const  			{ return m_init_negation; }
	        agnostic::Mutex_Set&    mutexes()                       { return m_mutexes; }

	    /** chao add
	     *
	     * @param f
	     * @return
	     */
        std::vector<const Action*>&
        actions_negation_adding( unsigned f )		{ return m_negation_adding[f]; }
        std::vector< std::pair< unsigned, const Action*> >&
        ceffs_negation_adding( unsigned f )		{ return m_negation_ceffs_adding[f]; }
        std::vector<const Action*>&
        actions_negation_deleting( unsigned f )		{ return m_negation_deleting[f]; }


        std::vector<const Action*>&
        actions_negation_edeleting( unsigned f )		{ return m_negation_edeleting[f]; }
        std::vector<const Action*>&
        actions_negation_bwd_edeleting( unsigned f )		{ return m_negation_bwd_edeleting[f]; }

        std::vector<const Action*>&
        actions_negation_requiring( unsigned f )		{ return m_negation_requiring[f]; }
        const std::vector<const Action*>&
        actions_negation_adding( unsigned f ) const	{ return m_negation_adding[f]; }
        const std::vector<const Action*>&
        actions_negation_deleting( unsigned f ) const	{ return m_negation_deleting[f]; }
        const std::vector<const Action*>&
        actions_negation_edeleting( unsigned f ) const	{ return m_negation_edeleting[f]; }
        const std::vector<const Action*>&
        actions_negation_bwd_edeleting( unsigned f ) const	{ return m_negation_bwd_edeleting[f]; }
        const std::vector<const Action*>&
        actions_negation_requiring( unsigned f ) const	{ return m_negation_requiring[f]; }
        /** chao add end
         *
         * @param f
         * @return
         */

		std::vector<const Action*>&		
		 			actions_adding( unsigned f )		{ return m_adding[f]; }

		std::vector< std::pair< unsigned, const Action*> >&
					ceffs_adding( unsigned f )		{ return m_ceffs_adding[f]; }

		const std::vector< std::pair< unsigned, const Action*> >&
					ceffs_adding( unsigned f ) const	{ return m_ceffs_adding[f]; }

		std::vector<const Action*>&		
		 			actions_deleting( unsigned f )		{ return m_deleting[f]; }


		std::vector<const Action*>&		
					actions_edeleting( unsigned f )		{ return m_edeleting[f]; }
        std::vector<const Action*>&
            actions_bwd_edeleting( unsigned f )		{ return m_bwd_edeleting[f]; }

		std::vector<const Action*>&		
					actions_requiring( unsigned f )		{ return m_requiring[f]; }
		const std::vector<const Action*>&		
					actions_adding( unsigned f ) const	{ return m_adding[f]; }
		const std::vector<const Action*>&		
					actions_deleting( unsigned f ) const	{ return m_deleting[f]; }
		const std::vector<const Action*>&		
					actions_edeleting( unsigned f ) const	{ return m_edeleting[f]; }
        const std::vector<const Action*>&
                    actions_bwd_edeleting( unsigned f ) const	{ return m_bwd_edeleting[f]; }
		const std::vector<const Action*>&		
					actions_requiring( unsigned f ) const	{ return m_requiring[f]; }

		const std::vector<const Action*>&
					empty_prec_actions() const 		{ return m_empty_precs; }

		void			applicable_actions( const State& s, std::vector<const Action* >& actions  ) const {
			m_succ_gen.retrieve_applicable(s,actions);
		}
		
		void			applicable_actions( const State& s, std::vector<int>& actions ) const {
			m_succ_gen.retrieve_applicable(s,actions);
		}
		
		void			applicable_actions_v2( const State& s, std::vector<int>& actions ) const {
			m_succ_gen_v2.retrieve_applicable(s,actions);
		}

		void			applicable_actions( const std::vector<float>& v, std::vector<const Action*>& actions ) const {
			m_succ_gen.retrieve_applicable( v, actions );
		}

		bool			is_in_init( unsigned f )	{ return m_in_init[f]; }
		bool			is_in_init( unsigned f ) const	{ return m_in_init[f]; }
		bool			is_in_goal( unsigned f )	{ return m_in_goal[f]; }
		bool			is_in_goal( unsigned f ) const	{ return m_in_goal[f]; }
        bool			is_in_negation( unsigned f )	{ return m_in_init_negation[f]; }
        bool			is_in_negation( unsigned f ) const	{ return m_in_init_negation[f]; }
        std::vector<bool> get_init_negation()         {return m_in_init_negation;}
        std::vector<bool> get_init_negation()     const    {return m_in_init_negation;}
		void                    print_fluent_vec(const Fluent_Vec &a);
		unsigned                end_operator() { return m_end_operator_id; }
      	        unsigned                end_operator() const { return m_end_operator_id; }
	        unsigned                get_fluent_index(std::string signature);

		void			make_action_tables(bool generate_match_tree = true);
		/** chao add
		 *
		 * @param generate_match_tree
		 */
        void			make_negation_action_tables(bool generate_match_tree = true);

		void			print( std::ostream& os ) const;
		void			print_fluents( std::ostream& os ) const;
		void			print_actions( std::ostream& os ) const;
		void                    print_action( unsigned idx, std::ostream& os ) const;
		void			print_fluent_vec( std::ostream& os, const Fluent_Vec& v ) const;	
		const agnostic::Successor_Generator&
					successor_generator() const { return m_succ_gen; }

	    void			initialize_successor_generator()  { m_succ_gen.build(); }

		bool			has_conditional_effects() const { return m_has_cond_effs; }
		void			notify_cond_eff_in_action() { m_has_cond_effs = true; }	

		// MRJ: Only to be used in the presence of conditional effects, as it
		// prevents the stronger inference that requires computing h^2
		void			compute_edeletes();

		void			set_verbose( bool v ) { m_verbose = v; }

		const std::vector< Best_Supporter >&	effects() const { return m_effects; }
		std::vector< Trigger >&			triggers()  const	{ return m_triggers; }
		/** chao add
		 *
		 * @param p
		 * @return
		 */
        const std::vector< Best_Supporter >&	negation_effects() const { return m_effects; }
        std::vector< Trigger >&			negation_triggers()  const	{ return m_triggers; }

		const std::set< unsigned >&		relevant_effects( unsigned p ) const { return m_relevant_effects[p]; }
		/** chao edit
		 *
		 * @param p
		 * @return
		 */
        const std::set< unsigned >&		relevant_negation_effects( unsigned p ) const { return m_negation_relevant_effects[p]; }

		void					make_effect_tables();
		/** chao add
		 *
		 */
        void					make_negation_effect_tables();

	protected:
	   /**chao_add
	    *
	    */
        void			increase_num_negation_fluents()        	{ m_negation_num_fluents++; }
        void			increase_num_negation_actions()        	{ m_negation_num_actions++; }
		void			increase_num_fluents()        	{ m_num_fluents++; }
		void			increase_num_actions()        	{ m_num_actions++; }
		void			register_action_in_tables( Action* act );
        void			register_negation_action_in_tables( Action* act );

	protected:

		std::string								m_domain_name;
		std::string								m_problem_name;
		unsigned		 						m_num_fluents;
		unsigned		 						m_num_actions;
		/** chao edit
		 *
		 */
        unsigned		 						m_negation_num_fluents;
        unsigned		 						m_negation_num_actions;
        std::string                             m_direction;
        std::string                             m_intersection;

//        Negation_Action_Ptr_Vec		 						m_negation_actions;
        Action_Ptr_Vec		 						m_negation_actions;
		Action_Ptr_Vec		 						m_actions;
        std::vector<const Action*>						m_const_actions;
//        std::vector<const Negation_Action*>						m_const_negation_actions;
        std::vector<const Action*>						m_const_negation_actions;
		Fluent_Ptr_Vec		 						m_fluents;
		/** chao add
		 *
		 */
//        Negation_Fluent_Ptr_Vec		 						m_negation_fluents;
//        std::vector<const Negation_Fluent*>						m_const_negation_fluents;
        Fluent_Ptr_Vec		 						m_negation_fluents;
        std::vector<const Fluent*>						m_const_negation_fluents;
		std::vector<const Fluent*>						m_const_fluents;
		Fluent_Vec		 						m_init;
		Fluent_Vec		 						m_goal;
		/** chao edit
		 *
		 */
        Fluent_Vec		 						m_init_negation;

        Fluent_Action_Table	 						m_negation_adding;
        Fluent_Action_Table	 						m_negation_requiring;
        Fluent_Action_Table	 						m_negation_deleting;
        Fluent_Action_Table	 						m_negation_edeleting;
        Fluent_Action_Table	 						m_negation_bwd_edeleting;
        std::vector< std::vector< std::pair< unsigned, const Action* > > >	m_negation_ceffs_adding;
        std::vector< std::set< unsigned> >					m_negation_relevant_effects;
        std::vector< Best_Supporter >						m_negation_effects;
        mutable std::vector< Trigger >						m_negation_triggers;

		Fluent_Action_Table	 						m_adding;
		Fluent_Action_Table	 						m_requiring;
		Fluent_Action_Table	 						m_deleting;
		Fluent_Action_Table	 						m_edeleting;
        Fluent_Action_Table	 						m_bwd_edeleting;
		std::vector<bool>	 						m_in_init;
		std::vector<bool>	 						m_in_goal;
		/** chao edit
		 *
		 */
        std::vector<bool>	 						    m_in_init_negation;
		unsigned                 						m_end_operator_id;
	  	std::map<std::string,int> 						m_fluents_map;
	  	/** chao edit
	  	 *
	  	 */
        std::map<std::string,int> 						m_negation_fluents_map;
		agnostic::Successor_Generator						m_succ_gen;
		agnostic::Match_Tree							m_succ_gen_v2;
		std::vector< const  Action* >   					m_empty_precs;
		std::vector< std::vector< std::pair< unsigned, const Action* > > >	m_ceffs_adding;
		bool									m_has_cond_effs;
		bool									m_verbose;
		std::vector< Best_Supporter >						m_effects;
		mutable std::vector< Trigger >						m_triggers;
		std::vector< std::set< unsigned> >					m_relevant_effects;
	        agnostic::Mutex_Set             	                                m_mutexes;

    };

}

#endif // STRIPS_Problem.hxx
