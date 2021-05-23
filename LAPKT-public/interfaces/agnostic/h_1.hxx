/*
Lightweight Automated Planning Toolkit
Copyright (C) 2012
Miquel Ramirez <miquel.ramirez@rmit.edu.au>
Nir Lipovetzky <nirlipo@gmail.com>
Christian Muise <christian.muise@gmail.com>

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

#ifndef __H_1__
#define __H_1__

#include <aptk/search_prob.hxx>
#include <aptk/heuristic.hxx>
#include <aptk/ext_math.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <boost/circular_buffer.hpp>
#include <vector>
#include <deque>

namespace aptk {

namespace agnostic {

class H_Max_Evaluation_Function  {

public:

	H_Max_Evaluation_Function( std::vector<float>& value_table)
	: m_values( value_table ) {
	}	

	float	operator()( Fluent_Vec::const_iterator begin, Fluent_Vec::const_iterator end, float v2 = 0.0f ) const {
		float v = v2;
		for ( Fluent_Vec::const_iterator it = begin; it != end; it++ ) {
			v = ( v < m_values[*it] ? m_values[*it] : v );
			//if ( v == infty ) return v;
		}
		return v;	
	}

private:

	const std::vector<float>&	m_values;
//    const std::vector<float>&	m_values_fwd;

};

class H_Add_Evaluation_Function {

public:
	H_Add_Evaluation_Function( std::vector<float>& value_table ) 
	: m_values( value_table ) {
	}	

	float	operator()( Fluent_Vec::const_iterator begin, Fluent_Vec::const_iterator end, float v2 = 0.0f ) const {
		float v = v2;
		for ( Fluent_Vec::const_iterator it = begin; it != end; it++ ) {
			if ( m_values[*it] == infty ) 
				return infty;
			v += m_values[*it];
		}
		return v;	
	}
private:

	const std::vector<float>&	m_values;

};

enum class H1_Cost_Function { Ignore_Costs, Use_Costs, LAMA};

template <typename bwd_Search_Problem, typename  Fwd_Search_Problem,typename Fluent_Set_Eval_Func, H1_Cost_Function cost_opt = H1_Cost_Function::Use_Costs >
class H1_Heuristic : public Heuristic<State> {
public:

	typedef STRIPS_Problem::Best_Supporter 	Best_Supporter;

	H1_Heuristic( const bwd_Search_Problem& prob_bwd,const Fwd_Search_Problem& prop_fwd )
	: Heuristic<State>( prob_bwd,prop_fwd ), m_strips_model_bwd( prob_bwd.task() ),m_strips_model_fwd(prop_fwd.task()) ,eval_func_bwd(m_values_bwd), eval_func_fwd(m_values_fwd){
		m_values_bwd.resize( m_strips_model_bwd.num_fluents() );
		m_difficulties_bwd.resize( m_strips_model_bwd.num_fluents() );
		m_best_supporters_bwd.resize(  m_strips_model_bwd.num_fluents() );
		m_already_updated_bwd.resize( m_strips_model_bwd.num_fluents() );
		m_allowed_actions_bwd.resize( m_strips_model_bwd.num_actions() );
		m_updated_bwd.resize( m_strips_model_bwd.num_fluents() );

        m_values_fwd.resize( m_strips_model_fwd.num_fluents() );
        m_difficulties_fwd.resize( m_strips_model_fwd.num_fluents() );
        m_best_supporters_fwd.resize(  m_strips_model_fwd.num_fluents() );
        m_already_updated_fwd.resize( m_strips_model_fwd.num_fluents() );
        m_allowed_actions_fwd.resize( m_strips_model_fwd.num_actions() );
        m_updated_fwd.resize( m_strips_model_fwd.num_fluents() );

		// HAZ: Set up the relevant actions once here so we don't need
		//      to iterate through all of them when evaluating.
		m_relevant_actions_fwd.resize( m_strips_model_fwd.num_fluents() );
        m_relevant_actions_bwd.resize( m_strips_model_bwd.num_fluents() );

		for ( unsigned i = 0; i < m_strips_model_bwd.num_actions(); i++ ) {

			const Action& a = *(m_strips_model_bwd.actions()[i]);

			// Relevant if the fluent is in the precondition
			for ( unsigned j = 0; j < a.prec_vec().size(); ++j ) {
				m_relevant_actions_bwd[a.prec_vec()[j]].insert(i);
			}

			// Relevant if the fluent is in the head of a conditional effect
			for ( unsigned j = 0; j < a.ceff_vec().size(); ++j ) {

				const Conditional_Effect& ceff = *(a.ceff_vec()[j]);

				for ( unsigned k = 0; k < ceff.prec_vec().size(); ++k) {
					m_relevant_actions_bwd[ceff.prec_vec()[k]].insert(i);
				}
			}
		}
        for ( unsigned i = 0; i < m_strips_model_fwd.num_actions(); i++ ) {

            const Action& a = *(m_strips_model_fwd.actions()[i]);

            // Relevant if the fluent is in the precondition
            for ( unsigned j = 0; j < a.prec_vec().size(); ++j ) {
                m_relevant_actions_fwd[a.prec_vec()[j]].insert(i);
            }

            // Relevant if the fluent is in the head of a conditional effect
            for ( unsigned j = 0; j < a.ceff_vec().size(); ++j ) {

                const Conditional_Effect& ceff = *(a.ceff_vec()[j]);

                for ( unsigned k = 0; k < ceff.prec_vec().size(); ++k) {
                    m_relevant_actions_fwd[ceff.prec_vec()[k]].insert(i);
                }
            }
        }
	}

	virtual ~H1_Heuristic() {
	}

	float 	value_bwd( unsigned p ) const { return m_values_bwd[p]; }
    float 	value_fwd( unsigned p ) const { return m_values_fwd[p]; }

	

        template <typename Search_Node>
        void eval( const Search_Node* n, float& h_val, std::vector<Action_Idx>& pref_ops) {
		eval(n->state(), h_val, pref_ops);				
	}

	
        template <typename Search_Node>
        void eval( const Search_Node* n, float& h_val ) {
		
		eval(n->state(),h_val);
	}
	
	virtual void eval_bwd( const Fluent_Vec& s, float& h_val ) {
		h_val = eval_func_bwd( s.begin(), s.end() );
	}
    virtual void eval_fwd( const Fluent_Vec& s, float& h_val ) {
        h_val = eval_func_fwd( s.begin(), s.end() );
    }

	template <typename Cost_Type>
	 void eval_bwd( const State& s, Cost_Type& h_out, State* goal = nullptr ) {
		float h;
		m_already_updated_bwd.reset();
		m_updated_bwd.clear();
		Fluent_Vec goal_vec;

		State* goal_tmp;


        if (goal== nullptr){
            goal_tmp=problem_fwd().init_state();
            if(s.fluent_vec().size()==m_strips_model_fwd.init().size()){
                const  Fluent_Vec* op_old  = &m_strips_model_fwd.init();
                Fluent_Vec* op = const_cast<Fluent_Vec*>(op_old);
                sort((*op).begin(),(*op).end());
                if (s.fluent_vec()==m_strips_model_fwd.init()){
                    goal_vec=m_strips_model_fwd.goal();
                }
            }else{
                goal_vec=s.fluent_vec();
            }
        } else{
            goal_tmp=goal;
            goal_vec=s.fluent_vec();
        }
		initialize_bwd(*goal_tmp);
		compute_bwd();



		h = eval_func_bwd( goal_vec.begin(), goal_vec.end() );
		h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
		
	}

    template <typename Cost_Type>
    void eval_fwd( const State& s, Cost_Type& h_out,State* goal = nullptr ) {
        float h;
        Fluent_Vec goal_vec;
        if(goal== nullptr){
            goal_vec=m_strips_model_fwd.goal();
        }
        else{
            goal_vec=goal->fluent_vec();
        }
        m_already_updated_fwd.reset();
        m_updated_fwd.clear();
        initialize_fwd(s);
        compute_fwd();
        h = eval_func_fwd( goal_vec.begin(), goal_vec.end() );
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;

    }
	/** chao edit
	 *
	 * @tparam Cost_Type
	 * @param s
	 * @param h_out
	 */
//    template <typename Cost_Type>
//    void eval_edit( const State& s, Cost_Type& h_out ) {
//        float h;
//        m_already_updated.reset();
//        m_updated.clear();
////        initialize_edit();
////        compute();
//        h = eval_func( s.fluent_vec().begin(), s.fluent_vec().end() );
//        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
//
//    }
	
	virtual void eval_reachability_bwd( const State& s, float& h_val, Fluent_Vec* persist_fluents = NULL ) {
		m_already_updated_bwd.reset();
		m_updated_bwd.clear();
		initialize_bwd(s);
		compute_reachability_bwd( persist_fluents );
		h_val = eval_func_bwd( m_strips_model_bwd.goal().begin(), m_strips_model_bwd.goal().end() );
		
	}
    virtual void eval_reachability_fwd( const State& s, float& h_val, Fluent_Vec* persist_fluents = NULL ) {
        m_already_updated_fwd.reset();
        m_updated_fwd.clear();
        initialize_fwd(s);
        compute_reachability_fwd( persist_fluents );
        h_val = eval_func_fwd( m_strips_model_fwd.goal().begin(), m_strips_model_fwd.goal().end() );

    }
    //later

	virtual void eval( const State& s, float& h_val,  std::vector<Action_Idx>& pref_ops ) {
		eval_bwd( s, h_val );
	}



	void print_values_bwd( std::ostream& os ) const {
		for ( unsigned p = 0; p < m_strips_model_bwd.fluents().size(); p++ ){
				os << "h1/add({ ";
				os << m_strips_model_bwd.fluents()[p]->signature();
				os << "}) = " << m_values_bwd[p] << std::endl;
			}		
	}
    void print_values_fwd( std::ostream& os ) const {
        for ( unsigned p = 0; p < m_strips_model_fwd.fluents().size(); p++ ){
            os << "h1/add({ ";
            os << m_strips_model_fwd.fluents()[p]->signature();
            os << "}) = " << m_values_fwd[p] << std::endl;
        }
    }

	Best_Supporter	get_best_supporter_bwd( unsigned f ) const {
		return m_best_supporters_bwd[f];
	}
    Best_Supporter	get_best_supporter_fwd( unsigned f ) const {
        return m_best_supporters_fwd[f];
    }
    
        void get_best_supporters_bwd( unsigned f, Action_Ptr_Const_Vec& bfs ) const {
		
		if(m_best_supporters_bwd[f].act_idx == no_such_index ) return;

	        const Action* bf = m_strips_model_bwd.actions()[ m_best_supporters_bwd[f].act_idx ];

		float h_val_bf = eval_func_bwd( bf->prec_vec().begin(), bf->prec_vec().end() );
		float h_val = 0;

		if ( !bf->asserts(f) ) { // added by conditional effect
			float min_cond_h = infty;
			for ( auto ceff : bf->ceff_vec() ) {
				if ( ceff->asserts( f ) ) {
					float h_cond = eval_func_bwd( ceff->prec_vec().begin(), ceff->prec_vec().end(), h_val_bf );
					if ( h_cond < min_cond_h )
						min_cond_h = h_cond;
				}
			}

			assert( !dequal(min_cond_h,infty) );
			h_val_bf = min_cond_h; 
		}

		const std::vector<const Action*>& add_acts = m_strips_model_bwd.actions_adding( f );

		for ( unsigned k = 0; k < add_acts.size(); k++ ){
			const Action* a = add_acts[k];
			h_val = eval_func_bwd( a->prec_vec().begin(), a->prec_vec().end() );
			if ( !a->asserts( f ) ) { // added by conditional effect
				float min_cond_h = infty;
				for ( auto ceff : a->ceff_vec() ) {
					if ( ceff->asserts(f) ) {
						float h_cond = eval_func_bwd( ceff->prec_vec().begin(), ceff->prec_vec().end(), h_val );
						if ( h_cond < min_cond_h )
							min_cond_h = h_cond;
					}
				}
				assert( !dequal(min_cond_h,infty) );
				h_val = min_cond_h;			
			}
			if ( dequal(h_val, h_val_bf ) )
				bfs.push_back(a);
		}
	}

    void get_best_supporters_fwd( unsigned f, Action_Ptr_Const_Vec& bfs ) const {

        if(m_best_supporters_fwd[f].act_idx == no_such_index ) return;

        const Action* bf = m_strips_model_fwd.actions()[ m_best_supporters_fwd[f].act_idx ];

        float h_val_bf = eval_func_fwd( bf->prec_vec().begin(), bf->prec_vec().end() );
        float h_val = 0;

        if ( !bf->asserts(f) ) { // added by conditional effect
            float min_cond_h = infty;
            for ( auto ceff : bf->ceff_vec() ) {
                if ( ceff->asserts( f ) ) {
                    float h_cond = eval_func_fwd( ceff->prec_vec().begin(), ceff->prec_vec().end(), h_val_bf );
                    if ( h_cond < min_cond_h )
                        min_cond_h = h_cond;
                }
            }

            assert( !dequal(min_cond_h,infty) );
            h_val_bf = min_cond_h;
        }

        const std::vector<const Action*>& add_acts = m_strips_model_fwd.actions_adding( f );

        for ( unsigned k = 0; k < add_acts.size(); k++ ){
            const Action* a = add_acts[k];
            h_val = eval_func_fwd( a->prec_vec().begin(), a->prec_vec().end() );
            if ( !a->asserts( f ) ) { // added by conditional effect
                float min_cond_h = infty;
                for ( auto ceff : a->ceff_vec() ) {
                    if ( ceff->asserts(f) ) {
                        float h_cond = eval_func_fwd( ceff->prec_vec().begin(), ceff->prec_vec().end(), h_val );
                        if ( h_cond < min_cond_h )
                            min_cond_h = h_cond;
                    }
                }
                assert( !dequal(min_cond_h,infty) );
                h_val = min_cond_h;
            }
            if ( dequal(h_val, h_val_bf ) )
                bfs.push_back(a);
        }
    }

protected:

	void	update_bwd( unsigned p, float v ) {
		if ( v >= m_values_bwd[p] ) return;
		m_values_bwd[p] = v;
		if ( !m_already_updated_bwd.isset( p ) ) {
			m_updated_bwd.push_back( p );
			m_already_updated_bwd.set( p );
		}		
	}
    void	update_fwd( unsigned p, float v ) {
        if ( v >= m_values_fwd[p] ) return;
        m_values_fwd[p] = v;
        if ( !m_already_updated_fwd.isset( p ) ) {
            m_updated_fwd.push_back( p );
            m_already_updated_fwd.set( p );
        }
    }

	void	update( unsigned p, float v, Best_Supporter bs ) {
		update( p, v, bs.act_idx, bs.eff_idx );
	}

	float eval_diff_bwd( const Best_Supporter& bs ) const {
		float min_val = infty;
		if ( bs.act_idx == no_such_index )
			return 0;
		const Action* a =  m_strips_model_bwd.actions()[bs.act_idx];
		for ( auto p : a->prec_vec() )
			min_val = std::min( min_val, m_values_bwd[p] );
		if ( bs.eff_idx == no_such_index ) return min_val;
		for ( auto p : a->ceff_vec()[bs.eff_idx]->prec_vec() )
			min_val = std::min( min_val, m_values_bwd[p] );
		return min_val;
	}
    float eval_diff_fwd( const Best_Supporter& bs ) const {
        float min_val = infty;
        if ( bs.act_idx == no_such_index )
            return 0;
        const Action* a =  m_strips_model_fwd.actions()[bs.act_idx];
        for ( auto p : a->prec_vec() )
            min_val = std::min( min_val, m_values_fwd[p] );
        if ( bs.eff_idx == no_such_index ) return min_val;
        for ( auto p : a->ceff_vec()[bs.eff_idx]->prec_vec() )
            min_val = std::min( min_val, m_values_fwd[p] );
        return min_val;
    }


	void	update_bwd( unsigned p, float v, unsigned act_idx, unsigned eff_idx ) {
		if ( v > m_values_bwd[p] ) return;
		if ( v >= 0.0f && dequal( v, m_values_bwd[p] ) ) {
			Best_Supporter candidate( act_idx, eff_idx );
			float candidate_diff = eval_diff_bwd( candidate );
			if ( candidate_diff < m_difficulties_bwd[p] ) {
				m_best_supporters_bwd[p] = candidate;
				m_difficulties_bwd[p] = candidate_diff;
			}
			return;
		}
		m_values_bwd[p] = v;
		if ( !m_already_updated_bwd.isset( p ) ) {
			m_updated_bwd.push_back( p );
			m_already_updated_bwd.set( p );
		}
		m_best_supporters_bwd[p].act_idx = act_idx;
		m_best_supporters_bwd[p].eff_idx = eff_idx;
		m_difficulties_bwd[p] = eval_diff_bwd( m_best_supporters_bwd[p] );
	}
    void	update_fwd( unsigned p, float v, unsigned act_idx, unsigned eff_idx ) {
        if ( v > m_values_fwd[p] ) return;
        if ( v >= 0.0f && dequal( v, m_values_fwd[p] ) ) {
            Best_Supporter candidate( act_idx, eff_idx );
            float candidate_diff = eval_diff_fwd( candidate );
            if ( candidate_diff < m_difficulties_fwd[p] ) {
                m_best_supporters_fwd[p] = candidate;
                m_difficulties_fwd[p] = candidate_diff;
            }
            return;
        }
        m_values_fwd[p] = v;
        if ( !m_already_updated_fwd.isset( p ) ) {
            m_updated_fwd.push_back( p );
            m_already_updated_fwd.set( p );
        }
        m_best_supporters_fwd[p].act_idx = act_idx;
        m_best_supporters_fwd[p].eff_idx = eff_idx;
        m_difficulties_fwd[p] = eval_diff_fwd( m_best_supporters_fwd[p] );
    }

	void	set_bwd( unsigned p, float v ) {
		m_values_bwd[p] = v;
		if ( !m_already_updated_bwd.isset( p ) ) {
			m_updated_bwd.push_back( p );
			m_already_updated_bwd.set( p );
		}		
	}
    void	set_fwd( unsigned p, float v ) {
        m_values_fwd[p] = v;
        if ( !m_already_updated_fwd.isset( p ) ) {
            m_updated_fwd.push_back( p );
            m_already_updated_fwd.set( p );
        }
    }

	void	initialize_bwd( const State& s )
	{
		for ( unsigned k = 0; k < m_strips_model_bwd.num_fluents(); k++ ) {
		        m_values_bwd[k] = m_difficulties_bwd[k] = infty;
			m_best_supporters_bwd[k] = Best_Supporter( no_such_index, no_such_index );
		}

		for ( unsigned k = 0; k < m_strips_model_bwd.empty_prec_actions().size(); k++ ) {
			const Action& a = *(m_strips_model_bwd.empty_prec_actions()[k]);
			float v =  ( cost_opt == H1_Cost_Function::Ignore_Costs ? 1.0f : 
					( cost_opt == H1_Cost_Function::Use_Costs ? (float)a.cost()  : 1.0f + (float)a.cost()) );
			
			for ( Fluent_Vec::const_iterator it = a.add_vec().begin();
				it != a.add_vec().end(); it++ )
			    update_bwd( *it, v, a.index(), no_such_index );
			// Conditional effects
			for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
			{
				const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
				if ( !ceff.prec_vec().empty() ) continue;
				float v_eff = v;
				for ( Fluent_Vec::const_iterator it = ceff.add_vec().begin();
					it != ceff.add_vec().end(); it++ )
				    update_bwd( *it, v_eff, a.index(), j );
			}
		}
	
		for ( Fluent_Vec::const_iterator it = s.fluent_vec().begin(); 
			it != s.fluent_vec().end(); it++ )
			set_bwd( *it, 0.0f );
	}
    void	initialize_fwd( const State& s )
    {
        for ( unsigned k = 0; k < m_strips_model_fwd.num_fluents(); k++ ) {
            m_values_fwd[k] = m_difficulties_fwd[k] = infty;
            m_best_supporters_fwd[k] = Best_Supporter( no_such_index, no_such_index );
        }

        for ( unsigned k = 0; k < m_strips_model_fwd.empty_prec_actions().size(); k++ ) {
            const Action& a = *(m_strips_model_fwd.empty_prec_actions()[k]);
            float v =  ( cost_opt == H1_Cost_Function::Ignore_Costs ? 1.0f :
                         ( cost_opt == H1_Cost_Function::Use_Costs ? (float)a.cost()  : 1.0f + (float)a.cost()) );

            for ( Fluent_Vec::const_iterator it = a.add_vec().begin();
                  it != a.add_vec().end(); it++ )
                update_fwd( *it, v, a.index(), no_such_index );
            // Conditional effects
            for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
            {
                const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
                if ( !ceff.prec_vec().empty() ) continue;
                float v_eff = v;
                for ( Fluent_Vec::const_iterator it = ceff.add_vec().begin();
                      it != ceff.add_vec().end(); it++ )
                    update_fwd( *it, v_eff, a.index(), j );
            }
        }

        for ( Fluent_Vec::const_iterator it = s.fluent_vec().begin();
              it != s.fluent_vec().end(); it++ )
            set_fwd( *it, 0.0f );
    }

#ifdef DEBUG
/** chao edit
 *
 */

//    void	initialize_edit( )
//    {
//        for ( unsigned k = 0; k < m_strips_model.num_fluents(); k++ ) {
//            m_values[k] = m_difficulties[k] = infty;
//            m_best_supporters[k] = Best_Supporter( no_such_index, no_such_index );
//        }
//
//        for ( unsigned k = 0; k < m_strips_model.empty_prec_actions().size(); k++ ) {
//            const Action& a = *(m_strips_model.empty_prec_actions()[k]);
//            float v =  ( cost_opt == H1_Cost_Function::Ignore_Costs ? 1.0f :
//                         ( cost_opt == H1_Cost_Function::Use_Costs ? (float)a.cost()  : 1.0f + (float)a.cost()) );
//
//            for ( Fluent_Vec::const_iterator it = a.add_vec().begin();
//                  it != a.add_vec().end(); it++ )
//                update( *it, v, a.index(), no_such_index );
//            // Conditional effects
//            for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
//            {
//                const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
//                if ( !ceff.prec_vec().empty() ) continue;
//                float v_eff = v;
//                for ( Fluent_Vec::const_iterator it = ceff.add_vec().begin();
//                      it != ceff.add_vec().end(); it++ )
//                    update( *it, v_eff, a.index(), j );
//            }
//        }
//
//        for ( Fluent_Vec::const_iterator it = m_strips_model.init().begin();
//              it != m_strips_model.init().end(); it++ )
//            set( *it, 0.0f );
//    }

#endif
	void	compute_bwd(  )
	{

		while ( !m_updated_bwd.empty() ) {

			unsigned p = m_updated_bwd.front();
			//std::cout << p << ". " << m_strips_model.fluents()[p]->signature() << " " << m_values[p] << std::endl;
			m_updated_bwd.pop_front();
			m_already_updated_bwd.unset(p);

			//Successor_Generator::Heuristic_Iterator it( m_values, m_strips_model.successor_generator().nodes() );
			//int i = it.first();
			//std::cout << "First action: " << i << std::endl;
			//while ( i != -1 ) {
			for ( std::set<unsigned>::iterator action_it = m_relevant_actions_bwd[p].begin(); action_it != m_relevant_actions_bwd[p].end(); ++action_it) {

				const Action& a = *(m_strips_model_bwd.actions()[*action_it]);

				float h_pre = eval_func_bwd( a.prec_vec().begin(), a.prec_vec().end() );

				if ( h_pre == infty ) continue;
				//assert( h_pre != infty );

				//std::cout << "Action " << *action_it << ". " << a.signature() << " relevant cost " << a.cost() << std::endl;

				float v = ( cost_opt == H1_Cost_Function::Ignore_Costs ?
						1.0f + h_pre :
						( cost_opt == H1_Cost_Function::Use_Costs ?
							(float)a.cost() + h_pre :
							1.0f + (float)a.cost() + h_pre
						) );

				for ( Fluent_Vec::const_iterator it = a.add_vec().begin();
					it != a.add_vec().end(); it++ )
				    update_bwd( *it, v, a.index(), no_such_index );
				// Conditional effects
				for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
				{
					const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
					float h_cond = eval_func_bwd( ceff.prec_vec().begin(), ceff.prec_vec().end(), h_pre );
					if ( h_cond == infty ) continue;
					float v_eff = ( cost_opt == H1_Cost_Function::Ignore_Costs ?
						1.0f + h_cond :
						( cost_opt == H1_Cost_Function::Use_Costs ?
							(float)a.cost() + h_cond :
							1.0f + (float)a.cost() + h_cond
						) );
					for ( Fluent_Vec::const_iterator it = ceff.add_vec().begin();
						it != ceff.add_vec().end(); it++ )
					    update_bwd( *it, v_eff, a.index(), j );
				}

				//i = it.next();
			}
		}
		//print_values(std::cout);
	}

    void	compute_fwd(  )
    {

        while ( !m_updated_fwd.empty() ) {

            unsigned p = m_updated_fwd.front();
            //std::cout << p << ". " << m_strips_model.fluents()[p]->signature() << " " << m_values[p] << std::endl;
            m_updated_fwd.pop_front();
            m_already_updated_fwd.unset(p);

            //Successor_Generator::Heuristic_Iterator it( m_values, m_strips_model.successor_generator().nodes() );
            //int i = it.first();
            //std::cout << "First action: " << i << std::endl;
            //while ( i != -1 ) {
            for ( std::set<unsigned>::iterator action_it = m_relevant_actions_fwd[p].begin(); action_it != m_relevant_actions_fwd[p].end(); ++action_it) {

                const Action& a = *(m_strips_model_fwd.actions()[*action_it]);

                float h_pre = eval_func_fwd( a.prec_vec().begin(), a.prec_vec().end() );

                if ( h_pre == infty ) continue;
                //assert( h_pre != infty );

                //std::cout << "Action " << *action_it << ". " << a.signature() << " relevant cost " << a.cost() << std::endl;

                float v = ( cost_opt == H1_Cost_Function::Ignore_Costs ?
                            1.0f + h_pre :
                            ( cost_opt == H1_Cost_Function::Use_Costs ?
                              (float)a.cost() + h_pre :
                              1.0f + (float)a.cost() + h_pre
                            ) );

                for ( Fluent_Vec::const_iterator it = a.add_vec().begin();
                      it != a.add_vec().end(); it++ )
                    update_fwd( *it, v, a.index(), no_such_index );
                // Conditional effects
                for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
                {
                    const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
                    float h_cond = eval_func_fwd( ceff.prec_vec().begin(), ceff.prec_vec().end(), h_pre );
                    if ( h_cond == infty ) continue;
                    float v_eff = ( cost_opt == H1_Cost_Function::Ignore_Costs ?
                                    1.0f + h_cond :
                                    ( cost_opt == H1_Cost_Function::Use_Costs ?
                                      (float)a.cost() + h_cond :
                                      1.0f + (float)a.cost() + h_cond
                                    ) );
                    for ( Fluent_Vec::const_iterator it = ceff.add_vec().begin();
                          it != ceff.add_vec().end(); it++ )
                        update_fwd( *it, v_eff, a.index(), j );
                }

                //i = it.next();
            }
        }
        //print_values(std::cout);
    }

    /***************
     * Old Version *
     ***************/
	/*void	compute_old(  )
	{
		while ( !m_updated.empty() ) {

			unsigned p = m_updated.front();
			//std::cout << p << ". " << m_strips_model.fluents()[p]->signature() << " " << m_values[p] << std::endl;
			m_updated.pop_front();
			m_already_updated.unset(p);

			//Successor_Generator::Heuristic_Iterator it( m_values, m_strips_model.successor_generator().nodes() );
			//int i = it.first();
			//std::cout << "First action: " << i << std::endl;
			//while ( i != -1 ) {
			for ( unsigned i = 0; i < m_strips_model.num_actions(); i++ ) {
				const Action& a = *(m_strips_model.actions()[i]);

				//std::cout << "Action considered: " << a.signature() << std::endl;
				bool relevant =  a.prec_set().isset(p);
				
				for ( unsigned j = 0; j < a.ceff_vec().size() && !relevant; j++ ) {
					const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
					relevant = relevant || ceff.prec_set().isset(p);
				}
				
				if ( !relevant ) {
					//i = it.next();
					continue;
				}

				float h_pre = eval_func( a.prec_vec().begin(), a.prec_vec().end() );

				if ( h_pre == infty ) continue;
				//assert( h_pre != infty );

				//std::cout << "Action " << i << ". " << a.signature() << " relevant" << std::endl;

				float v = ( cost_opt == H1_Cost_Function::Ignore_Costs ?  
						1.0f + h_pre :
						( cost_opt == H1_Cost_Function::Use_Costs ?
							(float)a.cost() + h_pre :
							1.0f + (float)a.cost() + h_pre
						) );

				for ( Fluent_Vec::const_iterator it = a.add_vec().begin();
					it != a.add_vec().end(); it++ )
					update( *it, v, m_strips_model.actions()[i] );
				// Conditional effects
				for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
				{
					const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
					float h_cond = eval_func( ceff.prec_vec().begin(), ceff.prec_vec().end(), h_pre );
					if ( h_cond == infty ) continue;
					float v_eff = ( cost_opt == H1_Cost_Function::Ignore_Costs ?  
						1.0f + h_cond :
						( cost_opt == H1_Cost_Function::Use_Costs ?
							(float)a.cost() + h_cond :
							1.0f + (float)a.cost() + h_cond
						) );
					for ( Fluent_Vec::const_iterator it = ceff.add_vec().begin();
						it != ceff.add_vec().end(); it++ )
						update( *it, v_eff, m_strips_model.actions()[i] );
				}

				//i = it.next();
			}
		}
	}*/
	
	void	compute_reachability_bwd( Fluent_Vec* persist_fluents = NULL )
	{
		std::vector< const Action*>::const_iterator it_a =  m_strips_model_bwd.actions().begin();
		for (Bool_Vec::iterator it = m_allowed_actions_bwd.begin(); it != m_allowed_actions_bwd.end(); it++, it_a++){
			//		for ( unsigned i = 0; i < m_strips_model.num_actions(); i++ ) {
			//m_allowed_actions[i]=true;
			*it = true;
			
			if(! persist_fluents ) continue;
			
			//const Action& a = *(m_strips_model.actions()[i]);

			/**
			 * If actions edel or adds fluent that has to persist, exclude action.
			 */

			for(unsigned p = 0; p < persist_fluents->size(); p++){
				unsigned fl = persist_fluents->at(p);
				if( (*it_a)->asserts( fl ) || (*it_a)->edeletes( fl ) ){
					//m_allowed_actions[i] = false;
					*it = false;
					break;
				}
			}
		}

		while ( !m_updated_bwd.empty() ) {

			unsigned p = m_updated_bwd.front();
			//std::cout << p << ". " << m_strips_model.fluents()[p]->signature() << " " << m_values[p] << std::endl;
			m_updated_bwd.pop_front();
			m_already_updated_bwd.unset(p);

			//Successor_Generator::Heuristic_Iterator it( m_values, m_strips_model.successor_generator().nodes() );
			//int i = it.first();
			//std::cout << "First action: " << i << std::endl;
			//while ( i != -1 ) {
			Bool_Vec::iterator it_allowed = m_allowed_actions_bwd.begin();
			for ( unsigned i = 0; i < m_strips_model_bwd.num_actions(); i++, it_allowed++ ) {
				const Action& a = *(m_strips_model_bwd.actions()[i]);
				
				//if( ! m_allowed_actions[ i ] )
				if(! *it_allowed )
					continue;

				//std::cout << "Action considered: " << a.signature() << std::endl;
				bool relevant =  a.prec_set().isset(p);
				
				for ( unsigned j = 0; j < a.ceff_vec().size() && !relevant; j++ ) {
					const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
					relevant = relevant || ceff.prec_set().isset(p);
				}
				
				if ( !relevant ) {
					//i = it.next();
					continue;
				}

				float h_pre = eval_func_bwd( a.prec_vec().begin(), a.prec_vec().end() );

				if ( h_pre == infty ) continue;
				//assert( h_pre != infty );

				//std::cout << "Action " << i << ". " << a.signature() << " relevant" << std::endl;

				float v = 0.0f;

				for ( Fluent_Vec::const_iterator it = a.add_vec().begin();
					it != a.add_vec().end(); it++ )
					update_bwd( *it, v );
				// Conditional effects
				for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
				{
					const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
					float h_cond = std::max(eval_func_bwd( ceff.prec_vec().begin(), ceff.prec_vec().end() ), h_pre);
					if ( h_cond == infty ) continue;
					float v_eff = 0.0f;
					for ( Fluent_Vec::const_iterator it = ceff.add_vec().begin();
						it != ceff.add_vec().end(); it++ )
						update_bwd( *it, v_eff );
				}

				//i = it.next();
			}
		}
	}

    void	compute_reachability_fwd( Fluent_Vec* persist_fluents = NULL )
    {
        std::vector< const Action*>::const_iterator it_a =  m_strips_model_fwd.actions().begin();
        for (Bool_Vec::iterator it = m_allowed_actions_fwd.begin(); it != m_allowed_actions_fwd.end(); it++, it_a++){
            //		for ( unsigned i = 0; i < m_strips_model.num_actions(); i++ ) {
            //m_allowed_actions[i]=true;
            *it = true;

            if(! persist_fluents ) continue;

            //const Action& a = *(m_strips_model.actions()[i]);

            /**
             * If actions edel or adds fluent that has to persist, exclude action.
             */

            for(unsigned p = 0; p < persist_fluents->size(); p++){
                unsigned fl = persist_fluents->at(p);
                if( (*it_a)->asserts( fl ) || (*it_a)->edeletes( fl ) ){
                    //m_allowed_actions[i] = false;
                    *it = false;
                    break;
                }
            }
        }

        while ( !m_updated_fwd.empty() ) {

            unsigned p = m_updated_fwd.front();
            //std::cout << p << ". " << m_strips_model.fluents()[p]->signature() << " " << m_values[p] << std::endl;
            m_updated_fwd.pop_front();
            m_already_updated_fwd.unset(p);

            //Successor_Generator::Heuristic_Iterator it( m_values, m_strips_model.successor_generator().nodes() );
            //int i = it.first();
            //std::cout << "First action: " << i << std::endl;
            //while ( i != -1 ) {
            Bool_Vec::iterator it_allowed = m_allowed_actions_fwd.begin();
            for ( unsigned i = 0; i < m_strips_model_fwd.num_actions(); i++, it_allowed++ ) {
                const Action& a = *(m_strips_model_fwd.actions()[i]);

                //if( ! m_allowed_actions[ i ] )
                if(! *it_allowed )
                    continue;

                //std::cout << "Action considered: " << a.signature() << std::endl;
                bool relevant =  a.prec_set().isset(p);

                for ( unsigned j = 0; j < a.ceff_vec().size() && !relevant; j++ ) {
                    const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
                    relevant = relevant || ceff.prec_set().isset(p);
                }

                if ( !relevant ) {
                    //i = it.next();
                    continue;
                }

                float h_pre = eval_func_fwd( a.prec_vec().begin(), a.prec_vec().end() );

                if ( h_pre == infty ) continue;
                //assert( h_pre != infty );

                //std::cout << "Action " << i << ". " << a.signature() << " relevant" << std::endl;

                float v = 0.0f;

                for ( Fluent_Vec::const_iterator it = a.add_vec().begin();
                      it != a.add_vec().end(); it++ )
                    update_fwd( *it, v );
                // Conditional effects
                for ( unsigned j = 0; j < a.ceff_vec().size(); j++ )
                {
                    const Conditional_Effect& ceff = *(a.ceff_vec()[j]);
                    float h_cond = std::max(eval_func_fwd( ceff.prec_vec().begin(), ceff.prec_vec().end() ), h_pre);
                    if ( h_cond == infty ) continue;
                    float v_eff = 0.0f;
                    for ( Fluent_Vec::const_iterator it = ceff.add_vec().begin();
                          it != ceff.add_vec().end(); it++ )
                        update_fwd( *it, v_eff );
                }

                //i = it.next();
            }
        }
    }
		
protected:

	//const STRIPS_Problem&			m_strips_model;
    const STRIPS_Problem&			m_strips_model_bwd;
    const STRIPS_Problem&			m_strips_model_fwd;

	//std::vector<float>			m_values;
    std::vector<float>			m_values_bwd;
    std::vector<float>			m_values_fwd;


//	std::vector<float>			m_difficulties;
    std::vector<float>			m_difficulties_bwd;
    std::vector<float>			m_difficulties_fwd;


	//Fluent_Set_Eval_Func			eval_func;
    Fluent_Set_Eval_Func			eval_func_bwd;
    Fluent_Set_Eval_Func			eval_func_fwd;


	//std::vector< Best_Supporter >		m_best_supporters;
    std::vector< Best_Supporter >		m_best_supporters_bwd;
    std::vector< Best_Supporter >		m_best_supporters_fwd;


	//std::vector<const Action*>		m_app_set;
    std::vector<const Action*>		m_app_set_bwd;
    std::vector<const Action*>		m_app_set_fwd;


	//std::vector< std::set<unsigned> > m_relevant_actions;
    std::vector< std::set<unsigned> > m_relevant_actions_bwd;
    std::vector< std::set<unsigned> > m_relevant_actions_fwd;

	//std::deque<unsigned> 			m_updated;
	//boost::circular_buffer<int>		m_updated;
    boost::circular_buffer<int>		m_updated_bwd;
    boost::circular_buffer<int>		m_updated_fwd;

	//Bit_Set					m_already_updated;
    Bit_Set					m_already_updated_bwd;
    Bit_Set					m_already_updated_fwd;


//	Bool_Vec                                m_allowed_actions;
    Bool_Vec                                m_allowed_actions_bwd;
    Bool_Vec                                m_allowed_actions_fwd;

};

}

}

#endif // h_add.hxx
