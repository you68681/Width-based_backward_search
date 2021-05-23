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

#ifndef __H_2__
#define __H_2__

#include <aptk/search_prob.hxx>
#include <aptk/heuristic.hxx>
#include <aptk/bit_set.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <boost/circular_buffer.hpp>
#include <vector>
#include <list>
#include <iosfwd>
#include "bwd_search_prob.hxx"

namespace aptk {

namespace agnostic {

namespace H2_Helper {
	
	inline int	pair_index( unsigned p, unsigned q ) {
		return ( p >= q ? p*(p+1)/2 + q : q*(q+1)/2 + p);
	}

}

enum class H2_Cost_Function { Zero_Costs, Unit_Costs, Use_Costs };

template <typename bwd_Search_Problem, typename Fwd_Search_Probelm, H2_Cost_Function cost_opt = H2_Cost_Function::Use_Costs >
class H2_Heuristic : public Heuristic<State> {

public:
	H2_Heuristic( const bwd_Search_Problem& prob_bwd,const Fwd_Search_Probelm& pro_fwd )
	: Heuristic<State>( prob_bwd,pro_fwd ), m_strips_model_bwd( prob_bwd.task() ),m_strips_model_fwd(pro_fwd.task()) {     //2020.8.30  16.12    这里没有这个构造函数，因为之前修改了
		unsigned F_bwd = m_strips_model_bwd.num_fluents();
		unsigned F_fwd = m_strips_model_fwd.num_fluents();

		m_values_bwd.resize( (F_bwd*F_bwd + F_bwd)/2 );
        m_values_fwd.resize( (F_fwd*F_fwd + F_fwd)/2 );

		m_op_values_bwd.resize( m_strips_model_bwd.num_actions() );
        m_op_values_fwd.resize( m_strips_model_fwd.num_actions() );


		m_interfering_ops_bwd.resize( F_bwd );
        m_interfering_ops_fwd.resize( F_fwd );

		for ( unsigned p = 0; p < m_interfering_ops_bwd.size(); p++ ) {
			m_interfering_ops_bwd[p] = new Bit_Set( m_strips_model_bwd.num_actions() );
			for ( unsigned op = 0; op < m_strips_model_bwd.num_actions(); op++ ) 	{
				const Action* op_ptr = m_strips_model_bwd.actions()[op];
				if ( op_ptr->add_set().isset( p ) || op_ptr->del_set().isset( p ) )
					m_interfering_ops_bwd[p]->set(op);
			}
		}
        for ( unsigned p = 0; p < m_interfering_ops_fwd.size(); p++ ) {
            m_interfering_ops_fwd[p] = new Bit_Set( m_strips_model_fwd.num_actions() );
            for ( unsigned op = 0; op < m_strips_model_fwd.num_actions(); op++ ) 	{
                const Action* op_ptr = m_strips_model_fwd.actions()[op];
                if ( op_ptr->add_set().isset( p ) || op_ptr->del_set().isset( p ) )
                    m_interfering_ops_fwd[p]->set(op);
            }
        }



        // NIR: Set up the relevant actions once here so we don't need
		//      to iterate through all of them when evaluating, similar to h1
		m_already_updated_bwd.resize( (F_bwd*F_bwd + F_bwd)/2 );
        m_already_updated_fwd.resize( (F_fwd*F_fwd + F_fwd)/2 );

		m_updated_bwd.resize( (F_bwd*F_bwd + F_bwd)/2 );
        m_updated_fwd.resize( (F_fwd*F_fwd + F_fwd)/2 );

		tmp_updated_bwd.resize((F_bwd*F_bwd+ F_bwd)/2);
        tmp_updated_fwd.resize((F_fwd*F_fwd+ F_fwd)/2);

		m_relevant_actions_bwd.resize( (F_bwd*F_bwd + F_bwd)/2 );
        m_relevant_actions_fwd.resize( (F_fwd*F_fwd + F_fwd)/2 );

		for ( unsigned i = 0; i < m_strips_model_bwd.num_actions(); i++ ) {

			const Action& a = *(m_strips_model_bwd.actions()[i]);

			// Relevant if the fluent is in the precondition
			for ( unsigned p = 0; p < a.prec_vec().size(); ++p ) {
				for ( unsigned q = p; q < a.prec_vec().size(); ++q ) {
					
					m_relevant_actions_bwd[ H2_Helper::pair_index( a.prec_vec()[p], a.prec_vec()[q] ) ].insert(i);
				}
			}

			// Relevant if the fluent is in the head of a conditional effect
			for ( unsigned j = 0; j < a.ceff_vec().size(); ++j ) {

				const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
				for ( unsigned p = 0; p < ceff.prec_vec().size(); ++p) {
					for ( unsigned q = p; q < ceff.prec_vec().size(); ++q) {
						m_relevant_actions_bwd[ H2_Helper::pair_index( ceff.prec_vec()[p], ceff.prec_vec()[q] ) ].insert(i);
					}
				}
			}
		}

        for ( unsigned i = 0; i < m_strips_model_fwd.num_actions(); i++ ) {

            const Action& a = *(m_strips_model_fwd.actions()[i]);

            // Relevant if the fluent is in the precondition
            for ( unsigned p = 0; p < a.prec_vec().size(); ++p ) {
                for ( unsigned q = p; q < a.prec_vec().size(); ++q ) {

                    m_relevant_actions_fwd[ H2_Helper::pair_index( a.prec_vec()[p], a.prec_vec()[q] ) ].insert(i);
                }
            }

            // Relevant if the fluent is in the head of a conditional effect
            for ( unsigned j = 0; j < a.ceff_vec().size(); ++j ) {

                const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
                for ( unsigned p = 0; p < ceff.prec_vec().size(); ++p) {
                    for ( unsigned q = p; q < ceff.prec_vec().size(); ++q) {
                        m_relevant_actions_fwd[ H2_Helper::pair_index( ceff.prec_vec()[p], ceff.prec_vec()[q] ) ].insert(i);
                    }
                }
            }
        }


	}













	virtual ~H2_Heuristic() {
	}
	
	template <typename Search_Node>
        void eval( const Search_Node* n, float& h_val, std::vector<Action_Idx>& pref_ops) {
		eval(n->state(), h_val, pref_ops);				
	}

	
        template <typename Search_Node>
        void eval( const Search_Node* n, float& h_val ) {
		
		eval(n->state(),h_val);
	}
	
	virtual	void	eval_bwd( const State& s, float& h_val ) {
		m_already_updated_bwd.reset();
		m_updated_bwd.clear();
		initialize_bwd( s );
		compute_bwd();
		h_val = eval_bwd( m_strips_model_bwd.goal() );
    }

    virtual	void	eval_fwd( const State& s, float& h_val ) {
        m_already_updated_fwd.reset();
        m_updated_fwd.clear();
        initialize_fwd( s );
        compute_fwd();
        h_val = eval_fwd( m_strips_model_fwd.goal() );
    }

	virtual void eval_bwd( const State& s, float& h_val,  std::vector<Action_Idx>& pref_ops ) {
		eval_bwd( s, h_val );
	}
    virtual void eval_fwd( const State& s, float& h_val,  std::vector<Action_Idx>& pref_ops ) {
        eval_fwd( s, h_val );
    }

	float&	op_value_bwd( unsigned a ) 		{ return m_op_values_bwd.at(a); }
    float&	op_value_fwd( unsigned a ) 		{ return m_op_values_fwd.at(a); }

	float   op_value_bwd( unsigned a ) const 	{ return m_op_values_bwd.at(a); }
    float   op_value_fwd( unsigned a ) const 	{ return m_op_values_fwd.at(a); }

	float& value_bwd( unsigned p, unsigned q ) 	{
		assert( H2_Helper::pair_index(p,q) < (int)m_values_bwd.size() );
		return m_values_bwd[H2_Helper::pair_index(p,q)];
	}
    float& value_fwd( unsigned p, unsigned q ) 	{
        assert( H2_Helper::pair_index(p,q) < (int)m_values_fwd.size() );
        return m_values_fwd[H2_Helper::pair_index(p,q)];
    }


	float value_bwd( unsigned p, unsigned q ) const 	{
		assert( H2_Helper::pair_index(p,q) < (int)m_values_bwd.size() );
		return m_values_bwd[H2_Helper::pair_index(p,q)];
	}
    float value_fwd( unsigned p, unsigned q ) const 	{
        assert( H2_Helper::pair_index(p,q) < (int)m_values_fwd.size() );
        return m_values_fwd[H2_Helper::pair_index(p,q)];
    }


    float& value_bwd( unsigned p ) 	{
		assert( H2_Helper::pair_index(p,p) < (int)m_values_bwd.size() );
		return m_values_bwd[H2_Helper::pair_index(p,p)];
	}
    float& value_fwd( unsigned p ) 	{
        assert( H2_Helper::pair_index(p,p) < (int)m_values_fwd.size() );
        return m_values_fwd[H2_Helper::pair_index(p,p)];
    }

	float value_bwd( unsigned p ) const 	{
		assert( H2_Helper::pair_index(p,p) < (int)m_values_bwd.size() );
		return m_values_bwd[H2_Helper::pair_index(p,p)];
	}
    float value_fwd( unsigned p ) const 	{
        assert( H2_Helper::pair_index(p,p) < (int)m_values_fwd.size() );
        return m_values_fwd[H2_Helper::pair_index(p,p)];
    }

	float eval_bwd( const Fluent_Vec& s ) const {
		float v = 0;
		for ( unsigned i = 0; i < s.size(); i++ )
			for ( unsigned j = i; j < s.size(); j++ ){			
				v = std::max( v, value_bwd( s[i], s[j] ) );
				if(v == infty ) return infty;
			}

		return v;
	}
    float eval_fwd( const Fluent_Vec& s ) const {
        float v = 0;
        for ( unsigned i = 0; i < s.size(); i++ )
            for ( unsigned j = i; j < s.size(); j++ ){
                v = std::max( v, value_fwd( s[i], s[j] ) );
                if(v == infty ) return infty;
            }

        return v;
    }

	bool  is_mutex_bwd( const Fluent_Vec& s ) const {
		return eval_bwd(s) == infty;
	}
    bool  is_mutex_fwd( const Fluent_Vec& s ) const {
        return eval_fwd(s) == infty;
    }

	bool	is_mutex_bwd( unsigned p, unsigned q ) const {
		return value_bwd( p, q ) == infty;
	}
    bool	is_mutex_fwd( unsigned p, unsigned q ) const {
        return value_fwd( p, q ) == infty;
    }

	float eval_bwd( const Fluent_Vec& s, unsigned p ) const {
		float v = 0;
		for ( unsigned k = 0; k < s.size(); k++ )
			v = std::max( v, value_bwd( s[k], p ) );
		float v2 = 0;
		for ( unsigned i = 0; i < s.size(); i++ )
			for ( unsigned j = i; j < s.size(); j++ )
				v2 = std::max( v2, value_bwd( s[i], s[j] ) );
		return std::max( v, v2 );
	}
    float eval_fwd( const Fluent_Vec& s, unsigned p ) const {
        float v = 0;
        for ( unsigned k = 0; k < s.size(); k++ )
            v = std::max( v, value_fwd( s[k], p ) );
        float v2 = 0;
        for ( unsigned i = 0; i < s.size(); i++ )
            for ( unsigned j = i; j < s.size(); j++ )
                v2 = std::max( v2, value_fwd( s[i], s[j] ) );
        return std::max( v, v2 );
    }

	bool interferes_bwd( unsigned a, unsigned p ) const {
		return m_interfering_ops_bwd[p]->isset(a);
	}
    bool interferes_fwd( unsigned a, unsigned p ) const {
        return m_interfering_ops_fwd[p]->isset(a);
    }

	void print_values_bwd( std::ostream& os ) const {
		for ( unsigned p = 0; p < m_strips_model_bwd.fluents().size(); p++ )
			for ( unsigned q = p; q < m_strips_model_bwd.fluents().size(); q++ ) {
			    os << p << ", "<< q << " - ";
				os << "h²({ ";
				os << m_strips_model_bwd.fluents()[p]->signature();
				os << ", ";
				os << m_strips_model_bwd.fluents()[q]->signature();
				os << "}) = " << value_bwd(p,q) << std::endl;
			}

		for(auto a : m_strips_model_bwd.actions()){
		    os << a->index() << " - ";
            os << "h²({ ";
            os << a->signature();
            os << "}) = " << op_value_bwd(a->index()) << std::endl;
		}
	}
    void print_values_fwd( std::ostream& os ) const {
        for ( unsigned p = 0; p < m_strips_model_fwd.fluents().size(); p++ )
            for ( unsigned q = p; q < m_strips_model_fwd.fluents().size(); q++ ) {
                os << p << ", "<< q << " - ";
                os << "h²({ ";
                os << m_strips_model_fwd.fluents()[p]->signature();
                os << ", ";
                os << m_strips_model_fwd.fluents()[q]->signature();
                os << "}) = " << value_fwd(p,q) << std::endl;
            }

        for(auto a : m_strips_model_fwd.actions()){
            os << a->index() << " - ";
            os << "h²({ ";
            os << a->signature();
            os << "}) = " << op_value_fwd(a->index()) << std::endl;
        }
    }


	void compute_mutexes_only_bwd( const State& s ){
		initialize_bwd( s );
		compute_mutexes_only_bwd();
		
#ifdef DEBUG
		print_values(std::cout);
#endif
	}

    void compute_mutexes_only_fwd( const State& s ){
        initialize_fwd( s );
        compute_mutexes_only_fwd();

#ifdef DEBUG
        print_values(std::cout);
#endif
    }

	void compute_edeletes_bwd( STRIPS_Problem& prob ){
		m_already_updated_bwd.reset();
		m_updated_bwd.clear();
		initialize_bwd( prob.init() );
		compute_mutexes_only_bwd();
		extract_edeletes_bwd( prob );


#ifdef DEBUG
		print_values(std::cout);
		prob.print_actions(std::cout);
#endif
	}
    void compute_edit_bwd( STRIPS_Problem& prob,float& h_val ){
        m_already_updated_bwd.reset();
        m_updated_bwd.clear();
        initialize_bwd( prob.init() );
        compute_bwd();


#ifdef DEBUG
        print_values(std::cout);
		prob.print_actions(std::cout);
#endif
    }
	void compute_edeletes_aij_bwd( STRIPS_Problem& prob ){
		compute_mutexes_only_aij_bwd();
		extract_edeletes_bwd( prob );

		
#ifdef DEBUG
		print_values(std::cout);
		prob.print_actions(std::cout);
#endif
	}



    void compute_edeletes_fwd( STRIPS_Problem& prob ){
        m_already_updated_fwd.reset();
        m_updated_fwd.clear();
        initialize_fwd( prob.init() );
        compute_mutexes_only_fwd();
        extract_edeletes_fwd( prob );


#ifdef DEBUG
        print_values(std::cout);
		prob.print_actions(std::cout);
#endif
    }
    void compute_edit_fwd( STRIPS_Problem& prob,float& h_val ){
        m_already_updated_fwd.reset();
        m_updated_fwd.clear();
        initialize_fwd( prob.init() );
        compute_fwd();


#ifdef DEBUG
        print_values(std::cout);
		prob.print_actions(std::cout);
#endif
    }
    void compute_edeletes_aij_fwd( STRIPS_Problem& prob ){
        compute_mutexes_only_aij_fwd();
        extract_edeletes_fwd( prob );


#ifdef DEBUG
        print_values(std::cout);
		prob.print_actions(std::cout);
#endif
    }


	void 	goal_mutex_closure_bwd( STRIPS_Problem& prob ){
		unsigned fsize = prob.num_fluents();
		Fluent_Vec new_goal;
		for( auto g : prob.goal()  )
			new_goal.push_back(g);

		for( unsigned f_idx = 0; f_idx != fsize; f_idx++ ){
			Fluent* p =  prob.fluents()[f_idx];

			if( ! prob.is_in_init( p->index() ) ) continue;

			for( auto g : prob.goal()  ){
				if( is_mutex_bwd(p->index(),g) ){
					unsigned notp_idx = STRIPS_Problem::add_fluent( prob,"not-"+p->signature());
					new_goal.push_back(notp_idx);

					for( auto const_a : prob.actions_adding( p->index() ) ){
						Action* a = prob.actions()[const_a->index()];
						a->del_vec().push_back(notp_idx);
						a->del_set().set( notp_idx );
						//prob.actions_deleting( notp_idx ).push_back( a );
					}
					for( auto const_a : prob.actions_deleting( p->index() ) ){
						Action* a = prob.actions()[const_a->index()];
						a->add_vec().push_back(notp_idx);
						a->add_set().set( notp_idx );
						//prob.actions_adding( notp_idx ).push_back( a );
					}
					
					
					break;
				}			       
			}
		}
		//STRIPS_Problem::set_goal( prob, new_goal );
		prob.make_action_tables(false);
		
	}

    void 	goal_mutex_closure_fwd( STRIPS_Problem& prob ){
        unsigned fsize = prob.num_fluents();
        Fluent_Vec new_goal;
        for( auto g : prob.goal()  )
            new_goal.push_back(g);

        for( unsigned f_idx = 0; f_idx != fsize; f_idx++ ){
            Fluent* p =  prob.fluents()[f_idx];

            if( ! prob.is_in_init( p->index() ) ) continue;

            for( auto g : prob.goal()  ){
                if( is_mutex_fwd(p->index(),g) ){
                    unsigned notp_idx = STRIPS_Problem::add_fluent( prob,"not-"+p->signature());
                    new_goal.push_back(notp_idx);

                    for( auto const_a : prob.actions_adding( p->index() ) ){
                        Action* a = prob.actions()[const_a->index()];
                        a->del_vec().push_back(notp_idx);
                        a->del_set().set( notp_idx );
                        //prob.actions_deleting( notp_idx ).push_back( a );
                    }
                    for( auto const_a : prob.actions_deleting( p->index() ) ){
                        Action* a = prob.actions()[const_a->index()];
                        a->add_vec().push_back(notp_idx);
                        a->add_set().set( notp_idx );
                        //prob.actions_adding( notp_idx ).push_back( a );
                    }


                    break;
                }
            }
        }
        //STRIPS_Problem::set_goal( prob, new_goal );
        prob.make_action_tables(false);

    }

	void
	compute_mutexes_only_aij_bwd() {
		for ( unsigned k = 0; k < m_values_bwd.size(); k++ )
			m_values_bwd[k] = 0.0f;
		typedef std::pair< unsigned, unsigned > Fluent_Pair;
		typedef std::list< Fluent_Pair > Pair_List;
		Pair_List M;

		// M_A set: pairs p,q s.t. there is at least one a where Add(a) = p and Del(a) = q

		for ( auto a : m_strips_model_bwd.actions() )
			for ( auto p : a->add_vec() )
				for ( auto q : a->del_vec() ) {
					if ( value_bwd(p,q) == infty ) continue;
					M.push_back( Fluent_Pair(p,q) );
					value_bwd(p,q) = infty;
				}
		
		Pair_List M_B;
		// M_B set: 
		
		for ( auto P : M ) {
			for ( auto a : m_strips_model_bwd.actions() ) {
				unsigned p = P.first;
				unsigned q = P.second;
				if ( !a->asserts(p) ) continue;
				for ( auto r : a->prec_vec() ) {
					if ( value_bwd(r,q) == infty ) continue;
					M_B.push_back( Fluent_Pair(r,q) );
					value_bwd(r, q) = infty;
				}
			}
		}

		for ( auto P : M_B )
			M.push_back( P );


		bool changed;
	
		do {
			changed = false;
			
			Pair_List M2;

			for ( auto P : M ) {
				unsigned p = P.first;
				unsigned q = P.second;
				if ( m_strips_model_bwd.is_in_init( p ) && m_strips_model_bwd.is_in_init(q) ) {
					value_bwd(p,q) = 0.0f;
					changed = true;
					continue;
				} 

				bool needs_to_be_removed = false;
				for ( auto adding : m_strips_model_bwd.actions_adding( p ) ) {
					bool is_deleting = adding->retracts(q);
					bool is_not_adding = !adding->asserts(q);
					if ( !is_deleting && !is_not_adding ) {
						needs_to_be_removed = true;
						break;
					}
					bool r_q_in_M = false;
					for ( auto r : adding->prec_vec() ) {
						if ( value_bwd(r,q) == infty ) {
							r_q_in_M = true;
							break;
						}
					}
					if ( !r_q_in_M ) {
						needs_to_be_removed = true;
						break;
					}
				}
				if ( needs_to_be_removed ) {
					value_bwd(p,q) = 0.0f;
					changed = true;
				}
				else {
					M2.push_back( P );
				}
			}
			M = M2;
 
		} while (changed);

	}
    void
    compute_mutexes_only_aij_fwd() {
        for ( unsigned k = 0; k < m_values_fwd.size(); k++ )
            m_values_fwd[k] = 0.0f;
        typedef std::pair< unsigned, unsigned > Fluent_Pair;
        typedef std::list< Fluent_Pair > Pair_List;
        Pair_List M;

        // M_A set: pairs p,q s.t. there is at least one a where Add(a) = p and Del(a) = q

        for ( auto a : m_strips_model_fwd.actions() )
            for ( auto p : a->add_vec() )
                for ( auto q : a->del_vec() ) {
                    if ( value_fwd(p,q) == infty ) continue;
                    M.push_back( Fluent_Pair(p,q) );
                    value_fwd(p,q) = infty;
                }

        Pair_List M_B;
        // M_B set:

        for ( auto P : M ) {
            for ( auto a : m_strips_model_fwd.actions() ) {
                unsigned p = P.first;
                unsigned q = P.second;
                if ( !a->asserts(p) ) continue;
                for ( auto r : a->prec_vec() ) {
                    if ( value_fwd(r,q) == infty ) continue;
                    M_B.push_back( Fluent_Pair(r,q) );
                    value_fwd(r, q) = infty;
                }
            }
        }

        for ( auto P : M_B )
            M.push_back( P );


        bool changed;

        do {
            changed = false;

            Pair_List M2;

            for ( auto P : M ) {
                unsigned p = P.first;
                unsigned q = P.second;
                if ( m_strips_model_fwd.is_in_init( p ) && m_strips_model_fwd.is_in_init(q) ) {
                    value_fwd(p,q) = 0.0f;
                    changed = true;
                    continue;
                }

                bool needs_to_be_removed = false;
                for ( auto adding : m_strips_model_fwd.actions_adding( p ) ) {
                    bool is_deleting = adding->retracts(q);
                    bool is_not_adding = !adding->asserts(q);
                    if ( !is_deleting && !is_not_adding ) {
                        needs_to_be_removed = true;
                        break;
                    }
                    bool r_q_in_M = false;
                    for ( auto r : adding->prec_vec() ) {
                        if ( value_fwd(r,q) == infty ) {
                            r_q_in_M = true;
                            break;
                        }
                    }
                    if ( !r_q_in_M ) {
                        needs_to_be_removed = true;
                        break;
                    }
                }
                if ( needs_to_be_removed ) {
                    value_fwd(p,q) = 0.0f;
                    changed = true;
                }
                else {
                    M2.push_back( P );
                }
            }
            M = M2;

        } while (changed);

    }

protected:

	void extract_edeletes_bwd( STRIPS_Problem& prob ){
		for ( unsigned p = 0 ; p < m_strips_model_bwd.num_fluents(); p++ ) {
            for (unsigned a = 0; a < m_strips_model_bwd.num_actions(); a++) {
                bool is_edelete = false;

                Action &action = *(prob.actions()[a]);

                for (unsigned i = 0; i < action.add_vec().size(); i++) {
                    unsigned q = action.add_vec()[i];
                    if (value_bwd(p, q) == infty) {
                        is_edelete = true;
                        prob.actions()[a]->edel_vec().push_back(p);
                        prob.actions()[a]->edel_set().set(p);
                        prob.actions_edeleting(p).push_back((const Action *) &action);
                        break;
                    }
                }

                if (is_edelete) continue;

                for (unsigned i = 0; i < action.prec_vec().size(); i++) {
                    unsigned r = action.prec_vec()[i];
                    if (!action.add_set().isset(p) && value_bwd(p, r) == infty) {
                        prob.actions()[a]->edel_vec().push_back(p);
                        prob.actions()[a]->edel_set().set(p);
                        prob.actions_edeleting(p).push_back((const Action *) &action);
                        break;
                    }
                }

                if (!action.edel_set().isset(p) && action.del_set().isset(p)) {
                    prob.actions()[a]->edel_vec().push_back(p);
                    prob.actions()[a]->edel_set().set(p);
                    prob.actions_edeleting(p).push_back((const Action *) &action);
                }

            }
            for (unsigned a = 0; a < m_strips_model_bwd.num_actions(); a++) {
                bool is_edelete = false;
                Action &action = *(prob.actions()[a]);
                //backward edel (regression)
                for (unsigned i = 0; i < action.prec_vec().size(); i++) {
                    unsigned q = action.prec_vec()[i];
                    if (value_bwd(p, q) == infty) {
                        is_edelete = true;
                        prob.actions()[a]->bwd_edel_vec().push_back(p);
                        prob.actions()[a]->bwd_edel_set().set(p);
                        prob.actions_bwd_edeleting(p).push_back((const Action *) &action);
                        break;
                    }
                }
                if (is_edelete) continue;

                for (unsigned i = 0; i < action.add_vec().size(); i++) {
                    unsigned r = action.add_vec()[i];
                    if (!action.prec_set().isset(p) && value_bwd(p, r) == infty) {
                        prob.actions()[a]->bwd_edel_vec().push_back(p);
                        prob.actions()[a]->bwd_edel_set().set(p);
                        prob.actions_bwd_edeleting(p).push_back((const Action *) &action);
                        break;
                    }
                }

                if (!action.edel_set().isset(p) && action.add_set().isset(p)) {
                    prob.actions()[a]->bwd_edel_vec().push_back(p);
                    prob.actions()[a]->bwd_edel_set().set(p);
                    prob.actions_bwd_edeleting(p).push_back((const Action *) &action);
                }
            }
        }
	}
    void extract_edeletes_fwd( STRIPS_Problem& prob ){
        for ( unsigned p = 0 ; p < m_strips_model_fwd.num_fluents(); p++ ) {
            for (unsigned a = 0; a < m_strips_model_fwd.num_actions(); a++) {
                bool is_edelete = false;
                /**
                 * 这里的一个变量_bwd_，跟函数名不符
                 *
                 */

                Action &action = *(prob.actions()[a]);

                for (unsigned i = 0; i < action.add_vec().size(); i++) {
                    unsigned q = action.add_vec()[i];
                    if (value_fwd(p, q) == infty) {
                        is_edelete = true;
                        prob.actions()[a]->edel_vec().push_back(p);
                        prob.actions()[a]->edel_set().set(p);
                        prob.actions_edeleting(p).push_back((const Action *) &action);
                        break;
                    }
                }

                if (is_edelete) continue;

                for (unsigned i = 0; i < action.prec_vec().size(); i++) {
                    unsigned r = action.prec_vec()[i];
                    if (!action.add_set().isset(p) && value_fwd(p, r) == infty) {
                        prob.actions()[a]->edel_vec().push_back(p);
                        prob.actions()[a]->edel_set().set(p);
                        prob.actions_edeleting(p).push_back((const Action *) &action);
                        break;
                    }
                }

                if (!action.edel_set().isset(p) && action.del_set().isset(p)) {
                    prob.actions()[a]->edel_vec().push_back(p);
                    prob.actions()[a]->edel_set().set(p);
                    prob.actions_edeleting(p).push_back((const Action *) &action);
                }

            }
            for (unsigned a = 0; a < m_strips_model_fwd.num_actions(); a++) {
                bool is_edelete = false;
                /**
                 * 这里的一个变量_bwd_，跟函数名不符
                 *
                 */
                Action &action = *(prob.actions()[a]);
                //backward edel (regression)
                for (unsigned i = 0; i < action.prec_vec().size(); i++) {
                    unsigned q = action.prec_vec()[i];
                    if (value_fwd(p, q) == infty) {
                        is_edelete = true;
                        /**
                        * 这里的一个变量_bwd_，跟函数名不符
                        *
                        */
                        prob.actions()[a]->edel_vec().push_back(p);
                        prob.actions()[a]->edel_set().set(p);
                        prob.actions_edeleting(p).push_back((const Action *) &action);
                        break;
                    }
                }
                if (is_edelete) continue;

                for (unsigned i = 0; i < action.add_vec().size(); i++) {
                    unsigned r = action.add_vec()[i];
                    if (!action.prec_set().isset(p) && value_fwd(p, r) == infty) {

                        /**
                        * 这里的一个变量_bwd_，跟函数名不符
                        *
                        */
                        prob.actions()[a]->edel_vec().push_back(p);
                        prob.actions()[a]->edel_set().set(p);
                        prob.actions_edeleting(p).push_back((const Action *) &action);
                        break;
                    }
                }

                if (!action.edel_set().isset(p) && action.add_set().isset(p)) {

                        /**
                        * 这里的一个变量_bwd_，跟函数名不符
                        *
                        */
                    prob.actions()[a]->edel_vec().push_back(p);
                    prob.actions()[a]->edel_set().set(p);
                    prob.actions_edeleting(p).push_back((const Action *) &action);
                }
            }
        }
    }


	void initialize_ceffs_and_emtpy_precs_bwd(){
		//conditional effects and empty precs
		for ( unsigned k = 0; k < m_strips_model_bwd.empty_prec_actions().size(); k++ ) {
			const Action& a = *(m_strips_model_bwd.empty_prec_actions()[k]);
			float v =  ( cost_opt == H2_Cost_Function::Unit_Costs ? 1.0f : 
					( cost_opt == H2_Cost_Function::Use_Costs ? (float)a.cost()  : 1.0f + (float)a.cost()) );
			
			for ( unsigned i = 0; i< a.add_vec().size(); i++ ){
				for ( unsigned j = i; i < a.add_vec().size(); j++ ){
					unsigned p = a.add_vec()[i];
					unsigned q = a.add_vec()[j];
					value_bwd(p,q) = v;
					
					int curr_idx = H2_Helper::pair_index(p,q);
					if ( !m_already_updated_bwd.isset( curr_idx ) ) {
						m_updated_bwd.push_back( curr_idx );
						m_already_updated_bwd.set( curr_idx );
					}
				}
			}

			// Conditional effects
			for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
			{
				const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
				if ( !ceff.prec_vec().empty() ) continue;
				float v_eff = v;
				
				
				for ( unsigned i = 0; i< ceff.add_vec().size(); i++ ){
					for ( unsigned j = i; i < ceff.add_vec().size(); j++ ){
						unsigned p = ceff.add_vec()[i];
						unsigned q = ceff.add_vec()[j];
						value_bwd(p,q) = v_eff;
						
						int curr_idx = H2_Helper::pair_index(p,q);
						if ( !m_already_updated_bwd.isset( curr_idx ) ) {
							m_updated_bwd.push_back( curr_idx );
							m_already_updated_bwd.set( curr_idx );
						}
					}
				}	
				
			}
		}
	}
    void initialize_ceffs_and_emtpy_precs_fwd(){
        //conditional effects and empty precs
        for ( unsigned k = 0; k < m_strips_model_fwd.empty_prec_actions().size(); k++ ) {
            const Action& a = *(m_strips_model_fwd.empty_prec_actions()[k]);
            float v =  ( cost_opt == H2_Cost_Function::Unit_Costs ? 1.0f :
                         ( cost_opt == H2_Cost_Function::Use_Costs ? (float)a.cost()  : 1.0f + (float)a.cost()) );

            for ( unsigned i = 0; i< a.add_vec().size(); i++ ){
                for ( unsigned j = i; i < a.add_vec().size(); j++ ){
                    unsigned p = a.add_vec()[i];
                    unsigned q = a.add_vec()[j];
                    value_fwd(p,q) = v;

                    int curr_idx = H2_Helper::pair_index(p,q);
                    if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                        m_updated_fwd.push_back( curr_idx );
                        m_already_updated_fwd.set( curr_idx );
                    }
                }
            }

            // Conditional effects
            for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
            {
                const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
                if ( !ceff.prec_vec().empty() ) continue;
                float v_eff = v;


                for ( unsigned i = 0; i< ceff.add_vec().size(); i++ ){
                    for ( unsigned j = i; i < ceff.add_vec().size(); j++ ){
                        unsigned p = ceff.add_vec()[i];
                        unsigned q = ceff.add_vec()[j];
                        value_fwd(p,q) = v_eff;

                        int curr_idx = H2_Helper::pair_index(p,q);
                        if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                            m_updated_fwd.push_back( curr_idx );
                            m_already_updated_fwd.set( curr_idx );
                        }
                    }
                }

            }
        }
    }



	void initialize_bwd( const State& s ) {
		for ( unsigned k = 0; k < m_values_bwd.size(); k++ )
			m_values_bwd[k] = infty;
		for ( unsigned k = 0; k < m_op_values_bwd.size(); k++ )
			m_op_values_bwd[k] = infty;

		initialize_ceffs_and_emtpy_precs_bwd();

		for ( unsigned i = 0; i < s.fluent_vec().size(); i++ )
		{
			unsigned p = s.fluent_vec()[i];
			value_bwd(p,p) = 0.0f;

			int curr_idx = H2_Helper::pair_index(p,p);
			if ( !m_already_updated_bwd.isset( curr_idx ) ) {
				m_updated_bwd.push_back( curr_idx );
				m_already_updated_bwd.set( curr_idx );
			}

			for ( unsigned j = i+1; j < s.fluent_vec().size(); j++ )
			{
				unsigned q = s.fluent_vec()[j];
				value_bwd(p,q) = 0.0f;

				int curr_idx = H2_Helper::pair_index(p,q);
				if ( !m_already_updated_bwd.isset( curr_idx ) ) {
					m_updated_bwd.push_back( curr_idx );
					m_already_updated_bwd.set( curr_idx );
				}
			}
		}
	}
    void initialize_fwd( const State& s ) {
        for ( unsigned k = 0; k < m_values_fwd.size(); k++ )
            m_values_fwd[k] = infty;
        for ( unsigned k = 0; k < m_op_values_fwd.size(); k++ )
            m_op_values_fwd[k] = infty;

        initialize_ceffs_and_emtpy_precs_fwd();

        for ( unsigned i = 0; i < s.fluent_vec().size(); i++ )
        {
            unsigned p = s.fluent_vec()[i];
            value_fwd(p,p) = 0.0f;

            int curr_idx = H2_Helper::pair_index(p,p);
            if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                m_updated_fwd.push_back( curr_idx );
                m_already_updated_fwd.set( curr_idx );
            }

            for ( unsigned j = i+1; j < s.fluent_vec().size(); j++ )
            {
                unsigned q = s.fluent_vec()[j];
                value_fwd(p,q) = 0.0f;

                int curr_idx = H2_Helper::pair_index(p,q);
                if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                    m_updated_fwd.push_back( curr_idx );
                    m_already_updated_fwd.set( curr_idx );
                }
            }
        }
    }


	void initialize_bwd( const Fluent_Vec& f ) {
		for ( unsigned k = 0; k < m_values_bwd.size(); k++ )
			m_values_bwd[k] = infty;
		for ( unsigned k = 0; k < m_op_values_bwd.size(); k++ )
			m_op_values_bwd[k] = infty;

		initialize_ceffs_and_emtpy_precs_bwd();

		for ( unsigned i = 0; i < f.size(); i++ )
		{
			unsigned p = f[i];
			value_bwd(p,p) = 0.0f;

			int curr_idx = H2_Helper::pair_index(p,p);
			if ( !m_already_updated_bwd.isset( curr_idx ) ) {
				m_updated_bwd.push_back( curr_idx );
				m_already_updated_bwd.set( curr_idx );
			}

			for ( unsigned j = i+1; j < f.size(); j++ )
			{
				unsigned q = f[j];
				value_bwd(p,q) = 0.0f;

				int curr_idx = H2_Helper::pair_index(p,q);
				if ( !m_already_updated_bwd.isset( curr_idx ) ) {
					m_updated_bwd.push_back( curr_idx );
					m_already_updated_bwd.set( curr_idx );
				}
			}
		}
	}
    void initialize_fwd( const Fluent_Vec& f ) {
        for ( unsigned k = 0; k < m_values_fwd.size(); k++ )
            m_values_fwd[k] = infty;
        for ( unsigned k = 0; k < m_op_values_fwd.size(); k++ )
            m_op_values_fwd[k] = infty;

        initialize_ceffs_and_emtpy_precs_fwd();

        for ( unsigned i = 0; i < f.size(); i++ )
        {
            unsigned p = f[i];
            value_fwd(p,p) = 0.0f;

            int curr_idx = H2_Helper::pair_index(p,p);
            if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                m_updated_fwd.push_back( curr_idx );
                m_already_updated_fwd.set( curr_idx );
            }

            for ( unsigned j = i+1; j < f.size(); j++ )
            {
                unsigned q = f[j];
                value_fwd(p,q) = 0.0f;

                int curr_idx = H2_Helper::pair_index(p,q);
                if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                    m_updated_fwd.push_back( curr_idx );
                    m_already_updated_fwd.set( curr_idx );
                }
            }
        }
    }

	void compute_bwd() {
		while ( !m_updated_bwd.empty() ) {

			unsigned p = m_updated_bwd.front();
			
			m_updated_bwd.pop_front();
			m_already_updated_bwd.unset(p);
	
			for ( std::set<unsigned>::iterator action_it = m_relevant_actions_bwd[p].begin(); action_it != m_relevant_actions_bwd[p].end(); ++action_it) {

				const Action& action = *(m_strips_model_bwd.actions()[*action_it]);
				unsigned a = action.index();

				op_value_bwd(a) = eval_bwd( action.prec_vec() );
				if ( op_value_bwd(a) == infty ) continue;
				
				for ( unsigned i = 0; i < action.add_vec().size(); i++ ) {
					unsigned p = action.add_vec()[i];
					for ( unsigned j = i; j < action.add_vec().size(); j++ ) {
						unsigned q = action.add_vec()[j];
						float curr_value = value_bwd(p,q);
						if ( curr_value == 0.0f ) continue;
						float v = op_value_bwd(a);
						if ( cost_opt == H2_Cost_Function::Unit_Costs ) v += 1.0f;
						if ( cost_opt == H2_Cost_Function::Use_Costs ) v += action.cost();
						if ( v < curr_value ) {
							value_bwd(p,q) = v;
							int curr_idx = H2_Helper::pair_index(p,q);
							if ( !m_already_updated_bwd.isset( curr_idx ) ) {
								m_updated_bwd.push_back( curr_idx );
								m_already_updated_bwd.set( curr_idx );
							}
							curr_idx = H2_Helper::pair_index(p,p);
							if ( !m_already_updated_bwd.isset( curr_idx ) ) {
								m_updated_bwd.push_back( curr_idx );
								m_already_updated_bwd.set( curr_idx );
							}
							curr_idx = H2_Helper::pair_index(q,q);
							if ( !m_already_updated_bwd.isset( curr_idx ) ) {
								m_updated_bwd.push_back( curr_idx );
								m_already_updated_bwd.set( curr_idx );
							}
						}
					}

					for ( unsigned r = 0; r < m_strips_model_bwd.num_fluents(); r++ ) {
						if ( interferes_bwd( a, r ) || value_bwd( p, r ) == 0.0f ) continue;
						float h2_pre_noop = std::max( op_value_bwd(a), value_bwd(r,r) );
						if ( h2_pre_noop == infty ) continue;
						for ( unsigned j = 0; j < action.prec_vec().size(); j++ ) {
							unsigned s = action.prec_vec()[j];
							h2_pre_noop = std::max( h2_pre_noop, value_bwd(r,s) );
						}
						float v = h2_pre_noop;
						if ( cost_opt == H2_Cost_Function::Unit_Costs ) v += 1.0f;
						if ( cost_opt == H2_Cost_Function::Use_Costs ) v += action.cost();
						if ( v < value_bwd(p,r) )
						{
							value_bwd(p,r) = v;
							int curr_idx = H2_Helper::pair_index(p,r);
							if ( !m_already_updated_bwd.isset( curr_idx ) ) {
								m_updated_bwd.push_back( curr_idx );
								m_already_updated_bwd.set( curr_idx );
							}
							curr_idx = H2_Helper::pair_index(r,r);
							if ( !m_already_updated_bwd.isset( curr_idx ) ) {
								m_updated_bwd.push_back( curr_idx );
								m_already_updated_bwd.set( curr_idx );
							}
							curr_idx = H2_Helper::pair_index(p,p);
							if ( !m_already_updated_bwd.isset( curr_idx ) ) {
								m_updated_bwd.push_back( curr_idx );
								m_already_updated_bwd.set( curr_idx );
							}
						}													
					}

				}
			}
			
		}
		
	}
    void compute_fwd() {
        while ( !m_updated_fwd.empty() ) {

            unsigned p = m_updated_fwd.front();

            m_updated_fwd.pop_front();
            m_already_updated_fwd.unset(p);

            for ( std::set<unsigned>::iterator action_it = m_relevant_actions_fwd[p].begin(); action_it != m_relevant_actions_fwd[p].end(); ++action_it) {

                const Action& action = *(m_strips_model_fwd.actions()[*action_it]);
                unsigned a = action.index();

                op_value_fwd(a) = eval_fwd( action.prec_vec() );
                if ( op_value_fwd(a) == infty ) continue;

                for ( unsigned i = 0; i < action.add_vec().size(); i++ ) {
                    unsigned p = action.add_vec()[i];
                    for ( unsigned j = i; j < action.add_vec().size(); j++ ) {
                        unsigned q = action.add_vec()[j];
                        float curr_value = value_fwd(p,q);
                        if ( curr_value == 0.0f ) continue;
                        float v = op_value_fwd(a);
                        if ( cost_opt == H2_Cost_Function::Unit_Costs ) v += 1.0f;
                        if ( cost_opt == H2_Cost_Function::Use_Costs ) v += action.cost();
                        if ( v < curr_value ) {
                            value_fwd(p,q) = v;
                            int curr_idx = H2_Helper::pair_index(p,q);
                            if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                                m_updated_fwd.push_back( curr_idx );
                                m_already_updated_fwd.set( curr_idx );
                            }
                            curr_idx = H2_Helper::pair_index(p,p);
                            if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                                m_updated_fwd.push_back( curr_idx );
                                m_already_updated_fwd.set( curr_idx );
                            }
                            curr_idx = H2_Helper::pair_index(q,q);
                            if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                                m_updated_fwd.push_back( curr_idx );
                                m_already_updated_fwd.set( curr_idx );
                            }
                        }
                    }

                    for ( unsigned r = 0; r < m_strips_model_fwd.num_fluents(); r++ ) {
                        if ( interferes_fwd( a, r ) || value_fwd( p, r ) == 0.0f ) continue;
                        float h2_pre_noop = std::max( op_value_fwd(a), value_fwd(r,r) );
                        if ( h2_pre_noop == infty ) continue;
                        for ( unsigned j = 0; j < action.prec_vec().size(); j++ ) {
                            unsigned s = action.prec_vec()[j];
                            h2_pre_noop = std::max( h2_pre_noop, value_fwd(r,s) );
                        }
                        float v = h2_pre_noop;
                        if ( cost_opt == H2_Cost_Function::Unit_Costs ) v += 1.0f;
                        if ( cost_opt == H2_Cost_Function::Use_Costs ) v += action.cost();
                        if ( v < value_fwd(p,r) )
                        {
                            value_fwd(p,r) = v;
                            int curr_idx = H2_Helper::pair_index(p,r);
                            if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                                m_updated_fwd.push_back( curr_idx );
                                m_already_updated_fwd.set( curr_idx );
                            }
                            curr_idx = H2_Helper::pair_index(r,r);
                            if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                                m_updated_fwd.push_back( curr_idx );
                                m_already_updated_fwd.set( curr_idx );
                            }
                            curr_idx = H2_Helper::pair_index(p,p);
                            if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                                m_updated_fwd.push_back( curr_idx );
                                m_already_updated_fwd.set( curr_idx );
                            }
                        }
                    }

                }
            }

        }

    }



	void compute_mutexes_only_bwd() {

		while ( !m_updated_bwd.empty() ) {
				
			unsigned p = m_updated_bwd.front();
			
			m_updated_bwd.pop_front();
			m_already_updated_bwd.unset(p);
	
			for ( std::set<unsigned>::iterator action_it = m_relevant_actions_bwd[p].begin(); action_it != m_relevant_actions_bwd[p].end(); ++action_it) {

				const Action& action = *(m_strips_model_bwd.actions()[*action_it]);
				unsigned a = action.index();
				op_value_bwd(a) = eval_bwd( action.prec_vec() );
				if ( op_value_bwd(a) == infty ) continue;

				for ( unsigned i = 0; i < action.add_vec().size(); i++ ) {
					unsigned p = action.add_vec()[i];
					for ( unsigned j = i; j < action.add_vec().size(); j++ ) {
						unsigned q = action.add_vec()[j];
						float curr_value = value_bwd(p,q);

						if ( curr_value == 0.0f ) continue;
						value_bwd(p,q) = 0.0f;

						int curr_idx = H2_Helper::pair_index(p,q);
						if ( !m_already_updated_bwd.isset( curr_idx ) ) {
							m_updated_bwd.push_back( curr_idx );
							m_already_updated_bwd.set( curr_idx );
						}
						curr_idx = H2_Helper::pair_index(p,p);
						if ( !m_already_updated_bwd.isset( curr_idx ) ) {
							m_updated_bwd.push_back( curr_idx );
							m_already_updated_bwd.set( curr_idx );
						}
						curr_idx = H2_Helper::pair_index(q,q);
						if ( !m_already_updated_bwd.isset( curr_idx ) ) {
							m_updated_bwd.push_back( curr_idx );
							m_already_updated_bwd.set( curr_idx );
						}
							
					}

					for ( unsigned r = 0; r < m_strips_model_bwd.num_fluents(); r++ ) {


						if ( interferes_bwd( a, r ) || value_bwd( p, r ) == 0.0f ) continue;
						float h2_pre_noop = std::max( op_value_bwd(a), value_bwd(r,r) );
						if ( h2_pre_noop == infty ) continue;

						for ( unsigned j = 0; j < action.prec_vec().size(); j++ ) {
                            unsigned s = action.prec_vec()[j];

                            h2_pre_noop = std::max(h2_pre_noop, value_bwd(r, s));
                        }

							/**
							if ( h2_pre_noop == infty ) {
                                if (H2_Helper::pair_index(p,r)== H2_Helper::pair_index(1,2))
                                {
                                    std::cout<<"find"<<std::endl;
                                }

							    //Check the reverse other: if action adding precondition s do not conflict with r, then value(r,s)==infy should be ignored.
							    //This may be an artifact of the relevant_actions datastructure and the action order propagation

                                for (auto act_add_s : m_strips_model.actions_adding(s)) {
                                    int as = act_add_s->index();
                                    if( ! interferes( as , r ) ){
                                        float h2_pre_s_noop = 0;
                                        for ( unsigned i = 0; i < act_add_s->prec_vec().size(); i++ ) {
                                            unsigned p = act_add_s->prec_vec()[i];
                                            for (unsigned j = i; j < act_add_s->prec_vec().size(); j++) {
                                                unsigned q = act_add_s->prec_vec()[j];
                                                float curr_value = value(p, q);
                                                if (curr_value==infty) {
                                                    h2_pre_s_noop=infty;
                                                    break;
                                                }
                                            }
                                        }

                                        //float h2_pre_s_noop = op_value(as);
                                        if (  h2_pre_s_noop == infty ) continue;

                                        for ( auto t : act_add_s->prec_vec() ) {
                                            h2_pre_s_noop = std::max( h2_pre_s_noop, value(t,r) );
                                            if (  h2_pre_s_noop == infty ) {
                                                h2_pre_noop =infty;
                                                break;
                                            }
                                        }

                                        //If the order of the propagation affect the result, then correct the he_pre_noop
                                        if(h2_pre_s_noop != infty) {
                                            h2_pre_noop = 0.0f;
                                            break;
                                        }

                                    }
                                }

                                 if (h2_pre_noop == infty){
                                     break;
                                 }


                            }
						}
						**/
						if ( h2_pre_noop == infty ) continue;

						value_bwd(p,r) = 0.0f;
                        int curr_idx = H2_Helper::pair_index(p,r);

//                        if (curr_idx==H2_Helper::pair_index(42, 74)){
//                            std::cout<<"find"<<std::endl;
//                        }
						if ( !m_already_updated_bwd.isset( curr_idx ) ) {
							m_updated_bwd.push_back( curr_idx );
							m_already_updated_bwd.set( curr_idx );
						}
						curr_idx = H2_Helper::pair_index(r,r);
						if ( !m_already_updated_bwd.isset( curr_idx ) ) {
							m_updated_bwd.push_back( curr_idx );
							m_already_updated_bwd.set( curr_idx );
						}
						/** new add
						 *
						 */
                        curr_idx = H2_Helper::pair_index(p,p);
                        if ( !m_already_updated_bwd.isset( curr_idx ) ) {
                            m_updated_bwd.push_back( curr_idx );
                            m_already_updated_bwd.set( curr_idx );
                        }


					}

				}
			}
		}


    }

    /**chao edit
     *

    void compute_mutexes_only() {
        std::vector<float>			m_values_record;
        bool different  = true;
        while (different || !tmp_updated.empty())  {

            m_values_record=m_values;
            tmp_updated.clear();

            //unsigned p = m_updated.front();

            //m_updated.pop_front();
            //m_already_updated.unset(p);
            for (int p : m_updated) {

                m_already_updated.unset( p);

                for (std::set<unsigned>::iterator action_it = m_relevant_actions[p].begin();
                     action_it != m_relevant_actions[p].end(); ++action_it) {

                    const Action &action = *(m_strips_model.actions()[*action_it]);
                    unsigned a = action.index();

                    op_value(a) = eval(action.prec_vec());
                    if (op_value(a) == infty) continue;

                    for (unsigned i = 0; i < action.add_vec().size(); i++) {
                        unsigned p = action.add_vec()[i];
                        for (unsigned j = i; j < action.add_vec().size(); j++) {
                            unsigned q = action.add_vec()[j];
                            float curr_value = value(p, q);
                            if (curr_value == 0.0f) continue;
                            value(p, q) = 0.0f;

                            int curr_idx = H2_Helper::pair_index(p, q);
                            if (!m_already_updated.isset(curr_idx)) {
                                tmp_updated.push_back(curr_idx);
                                m_already_updated.set(curr_idx);
                            }
                            curr_idx = H2_Helper::pair_index(p, p);
                            if (!m_already_updated.isset(curr_idx)) {
                                tmp_updated.push_back(curr_idx);
                                m_already_updated.set(curr_idx);
                            }
                            curr_idx = H2_Helper::pair_index(q, q);
                            if (!m_already_updated.isset(curr_idx)) {
                                tmp_updated.push_back(curr_idx);
                                m_already_updated.set(curr_idx);
                            }

                        }

                        for (unsigned r = 0; r < m_strips_model.num_fluents(); r++) {

                            if (interferes(a, r) || value(p, r) == 0.0f) continue;
                            float h2_pre_noop = std::max(op_value(a), value(r, r));
                            if (h2_pre_noop == infty) continue;

                            for (unsigned j = 0; j < action.prec_vec().size(); j++) {
                                unsigned s = action.prec_vec()[j];

                                h2_pre_noop = std::max(h2_pre_noop, value(r, s));

                                if (h2_pre_noop == infty) {

                                    //Check the reverse other: if action adding precondition s do not conflict with r, then value(r,s)==infy should be ignored.
                                    //This may be an artifact of the relevant_actions datastructure and the action order propagation

                                    for (auto act_add_s : m_strips_model.actions_adding(s)) {
                                        int as = act_add_s->index();
                                        if (!interferes(as, r)) {
                                            float h2_pre_s_noop = eval(act_add_s->prec_vec());
                                            //float h2_pre_s_noop = op_value(as);
                                            if (h2_pre_s_noop == infty) continue;

                                            for (auto t : act_add_s->prec_vec()) {
                                                h2_pre_s_noop = std::max(h2_pre_s_noop, value(t, r));
                                                if (h2_pre_s_noop == infty) {
                                                    h2_pre_noop = infty;
                                                    break;
                                                }
                                            }

                                            //If the order of the propagation affect the result, then correct the he_pre_noop
                                            if (h2_pre_s_noop != infty) {
                                                h2_pre_noop = 0.0f;
                                                break;
                                            }

                                        }
                                    }

                                    if (h2_pre_noop == infty) {
                                        break;
                                    }


                                }
                            }

                            if (h2_pre_noop == infty) continue;

                            value(p, r) = 0.0f;

                            int curr_idx = H2_Helper::pair_index(p, r);
                            if (!m_already_updated.isset(curr_idx)) {
                                tmp_updated.push_back(curr_idx);
                                m_already_updated.set(curr_idx);
                            }
                            curr_idx = H2_Helper::pair_index(r, r);
                            if (!m_already_updated.isset(curr_idx)) {
                                tmp_updated.push_back(curr_idx);
                                m_already_updated.set(curr_idx);
                            }
                        }

                    }
                }
            }
            if (compare(m_values,m_values_record)){
                different= false;

            }

            for (int p:tmp_updated) {
                m_updated.push_back(p);

            }

        }


    }
    */

    void compute_mutexes_only_fwd() {

        while ( !m_updated_fwd.empty() ) {

            unsigned p = m_updated_fwd.front();

            m_updated_fwd.pop_front();
            m_already_updated_fwd.unset(p);

            for ( std::set<unsigned>::iterator action_it = m_relevant_actions_fwd[p].begin(); action_it != m_relevant_actions_fwd[p].end(); ++action_it) {

                const Action& action = *(m_strips_model_fwd.actions()[*action_it]);
                unsigned a = action.index();
                op_value_fwd(a) = eval_fwd( action.prec_vec() );
                if ( op_value_fwd(a) == infty ) continue;

                for ( unsigned i = 0; i < action.add_vec().size(); i++ ) {
                    unsigned p = action.add_vec()[i];
                    for ( unsigned j = i; j < action.add_vec().size(); j++ ) {
                        unsigned q = action.add_vec()[j];
                        float curr_value = value_fwd(p,q);

                        if ( curr_value == 0.0f ) continue;
                        value_fwd(p,q) = 0.0f;

                        int curr_idx = H2_Helper::pair_index(p,q);
                        if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                            m_updated_fwd.push_back( curr_idx );
                            m_already_updated_fwd.set( curr_idx );
                        }
                        curr_idx = H2_Helper::pair_index(p,p);
                        if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                            m_updated_fwd.push_back( curr_idx );
                            m_already_updated_fwd.set( curr_idx );
                        }
                        curr_idx = H2_Helper::pair_index(q,q);
                        if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                            m_updated_fwd.push_back( curr_idx );
                            m_already_updated_fwd.set( curr_idx );
                        }

                    }

                    for ( unsigned r = 0; r < m_strips_model_fwd.num_fluents(); r++ ) {


                        if ( interferes_fwd( a, r ) || value_fwd( p, r ) == 0.0f ) continue;
                        float h2_pre_noop = std::max( op_value_fwd(a), value_fwd(r,r) );
                        if ( h2_pre_noop == infty ) continue;

                        for ( unsigned j = 0; j < action.prec_vec().size(); j++ ) {
                            unsigned s = action.prec_vec()[j];

                            h2_pre_noop = std::max(h2_pre_noop, value_fwd(r, s));
                        }

                        /**
                        if ( h2_pre_noop == infty ) {
                            if (H2_Helper::pair_index(p,r)== H2_Helper::pair_index(1,2))
                            {
                                std::cout<<"find"<<std::endl;
                            }

                            //Check the reverse other: if action adding precondition s do not conflict with r, then value(r,s)==infy should be ignored.
                            //This may be an artifact of the relevant_actions datastructure and the action order propagation

                            for (auto act_add_s : m_strips_model.actions_adding(s)) {
                                int as = act_add_s->index();
                                if( ! interferes( as , r ) ){
                                    float h2_pre_s_noop = 0;
                                    for ( unsigned i = 0; i < act_add_s->prec_vec().size(); i++ ) {
                                        unsigned p = act_add_s->prec_vec()[i];
                                        for (unsigned j = i; j < act_add_s->prec_vec().size(); j++) {
                                            unsigned q = act_add_s->prec_vec()[j];
                                            float curr_value = value(p, q);
                                            if (curr_value==infty) {
                                                h2_pre_s_noop=infty;
                                                break;
                                            }
                                        }
                                    }

                                    //float h2_pre_s_noop = op_value(as);
                                    if (  h2_pre_s_noop == infty ) continue;

                                    for ( auto t : act_add_s->prec_vec() ) {
                                        h2_pre_s_noop = std::max( h2_pre_s_noop, value(t,r) );
                                        if (  h2_pre_s_noop == infty ) {
                                            h2_pre_noop =infty;
                                            break;
                                        }
                                    }

                                    //If the order of the propagation affect the result, then correct the he_pre_noop
                                    if(h2_pre_s_noop != infty) {
                                        h2_pre_noop = 0.0f;
                                        break;
                                    }

                                }
                            }

                             if (h2_pre_noop == infty){
                                 break;
                             }


                        }
                    }
                    **/
                        if ( h2_pre_noop == infty ) continue;

                        value_fwd(p,r) = 0.0f;
                        int curr_idx = H2_Helper::pair_index(p,r);

//                        if (curr_idx==H2_Helper::pair_index(42, 74)){
//                            std::cout<<"find"<<std::endl;
//                        }
                        if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                            m_updated_fwd.push_back( curr_idx );
                            m_already_updated_fwd.set( curr_idx );
                        }
                        curr_idx = H2_Helper::pair_index(r,r);
                        if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                            m_updated_fwd.push_back( curr_idx );
                            m_already_updated_fwd.set( curr_idx );
                        }
                        /** new add
                         *
                         */
                        curr_idx = H2_Helper::pair_index(p,p);
                        if ( !m_already_updated_fwd.isset( curr_idx ) ) {
                            m_updated_fwd.push_back( curr_idx );
                            m_already_updated_fwd.set( curr_idx );
                        }


                    }

                }
            }
        }


    }



    bool compare(std::vector<float>vec_1,std::vector<float>vec_2){


        unsigned int size_1 = vec_1.size();
        unsigned int size_2 = vec_2.size();


        if( size_1 != size_2 ){
            return false;
        } else if ( size_1 == 0 ){
            return false;
        } else {
            for( 	std::vector<float>::iterator ite1 = vec_1.begin(), ite2 = vec_2.begin();
                    ite1 != vec_1.end(), ite2 != vec_2.end();
                    ite1++, ite2++
                    ){
                if( *ite1 != *ite2 ){
                    return false;
                    break;
                }
            }
        }
        return true;
    }
		


protected:
	
	//const STRIPS_Problem&			m_strips_model;
    const STRIPS_Problem&			m_strips_model_bwd;
    const STRIPS_Problem&			m_strips_model_fwd;

//	std::vector<float>			m_values;
    std::vector<float>			m_values_bwd;
    std::vector<float>			m_values_fwd;

//	std::vector<float>			m_op_values;
    std::vector<float>			m_op_values_bwd;
    std::vector<float>			m_op_values_fwd;

	//std::vector< Bit_Set* >			m_interfering_ops;
    std::vector< Bit_Set* >			m_interfering_ops_bwd;
    std::vector< Bit_Set* >			m_interfering_ops_fwd;

	//std::vector< std::set<unsigned> >       m_relevant_actions;
    std::vector< std::set<unsigned> >       m_relevant_actions_bwd;
    std::vector< std::set<unsigned> >       m_relevant_actions_fwd;

//	boost::circular_buffer<int>		m_updated;
    boost::circular_buffer<int>		m_updated_bwd;
    boost::circular_buffer<int>		m_updated_fwd;

//	Bit_Set					m_already_updated;
    Bit_Set					m_already_updated_bwd;
    Bit_Set					m_already_updated_fwd;

  //  boost::circular_buffer<int>		tmp_updated;
    boost::circular_buffer<int>		tmp_updated_bwd;
    boost::circular_buffer<int>		tmp_updated_fwd;

};

}

}
#endif // h_2.hxx
