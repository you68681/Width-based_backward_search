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

#ifndef __RP_HEURISTIC__
#define __RP_HEURISTIC__

#include <aptk/search_prob.hxx>
#include <aptk/heuristic.hxx>
#include <strips_state.hxx>
#include <types.hxx>
#include <action.hxx>
#include <fluent.hxx>
#include <queue>
#include <aptk/bit_array.hxx>
#include <vector>

namespace aptk {

namespace agnostic {

enum class RP_Cost_Function { Ignore_Costs, Use_Costs};
template < typename Primary_Heuristic, RP_Cost_Function cost_opt = RP_Cost_Function::Use_Costs >
class Relaxed_Plan_Extractor
{
	typedef std::queue<const Action* >	Action_Queue;
	typedef std::queue< const Fluent* >	Fluent_Queue;
public:

	Relaxed_Plan_Extractor( const STRIPS_Problem& prob_bwd,Primary_Heuristic& h_bwd, const STRIPS_Problem& prob_fwd,Primary_Heuristic& h_fwd )
		: m_base_heuristic_bwd(h_bwd),m_base_heuristic_fwd(h_fwd), m_strips_model_bwd( prob_bwd ), m_strips_model_fwd( prob_fwd ), m_ignore_rp_h_value_bwd(false), m_ignore_rp_h_value_fwd(false), m_one_ha_per_fluent_bwd(false),m_one_ha_per_fluent_fwd(false) {
		m_act_seen_bwd.resize( m_strips_model_bwd.num_actions() );
		m_init_fluents_bwd.resize( m_strips_model_bwd.num_fluents() );
		m_po_set_bwd.resize( m_strips_model_bwd.num_actions() );
		m_rp_precs_bwd.resize( m_strips_model_bwd.num_fluents() );

        m_act_seen_fwd.resize( m_strips_model_fwd.num_actions() );
        m_init_fluents_fwd.resize( m_strips_model_fwd.num_fluents() );
        m_po_set_fwd.resize( m_strips_model_fwd.num_actions() );
        m_rp_precs_fwd.resize( m_strips_model_fwd.num_fluents() );

	}

	virtual ~Relaxed_Plan_Extractor() {}

	
	void	set_one_HA_per_fluent_bwd( bool v) { m_one_ha_per_fluent_bwd = v; }
    void	set_one_HA_per_fluent_fwd( bool v) { m_one_ha_per_fluent_fwd = v; }

	bool	one_HA_per_fluent_bwd() const { return m_one_ha_per_fluent_bwd; }
    bool	one_HA_per_fluent_fwd() const { return m_one_ha_per_fluent_fwd; }

	bool	is_relaxed_plan_relevant_bwd( unsigned p ) const {
		return m_rp_precs_bwd.isset(p);
	}
    bool	is_relaxed_plan_relevant_fwd( unsigned p ) const {
        return m_rp_precs_fwd.isset(p);
    }

	/** chao edit
	 *
	 * @param s
	 * @param h_val
	 * @param pref_ops
	 * @param copy_rel_plan
	 * @param goals
	 */
    virtual void
    compute_edit_bwd( const State& s, float& h_val, std::vector<Action_Idx>& pref_ops, Fluent_Vec unachived_goal, std::vector<Action_Idx>* copy_rel_plan = NULL,   State* goal= NULL, Fluent_Vec* goals = NULL ) {
        h_val = 0.0;
        // 0. Initialize data structures
        actions_seen_bwd().reset();
        init_fluents_bwd().reset();
        if (m_strips_model_bwd.direction()=="f"){
            m_base_heuristic_bwd.eval_bwd( s, h_val,goal);
            if ( h_val == infty  )
                return;
        }

#if 0
        /** chao test
         * original from current to init
         * new from init to current
         */
//        for ( unsigned k = 0; k < s.fluent_vec().size(); k++ )
//            init_fluents().set(s.fluent_vec()[k] );
//        while ( !actions_pending().empty() )
//            actions_pending().pop();
//        std::vector<const Action*> relaxed_plan;
//        const Fluent_Vec& G = this->m_strips_model.init();
//
//        for ( unsigned k = 0; k < G.size(); k++ ) {
//
//            if ( init_fluents().isset( G[k] ) ) continue;
//
#endif
        if (goal!= nullptr){
            for ( unsigned k = 0; k < goal->fluent_vec().size(); k++ )
                init_fluents_bwd().set(goal->fluent_vec()[k] );
        }
        else{
            for ( unsigned k = 0; k < m_strips_model_bwd.init().size(); k++ )
                init_fluents_bwd().set(m_strips_model_bwd.init()[k] );
        }


        while ( !actions_pending_bwd().empty() )
            actions_pending_bwd().pop();

        std::vector<const Action*> relaxed_plan;
//       change the m_strips_model.goal() to init()
        const Fluent_Vec& G = s.fluent_vec();
//        const Fluent_Vec& G = goals ? *goals : m_strips_model.init();

        // 1. Add to the pending queue best supporters for goal fluents

        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_bwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_bwd.get_best_supporter_bwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
//                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_bwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_bwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_bwd().push( m_strips_model_bwd.fluents()[G[k]] );
            actions_pending_bwd().push( sup );
            actions_seen_bwd().set( sup->index() );
            relaxed_plan.push_back( sup );

        }

        m_rp_precs_bwd.reset();
//        for (unsigned p:m_strips_model.fluents()){
//            m_rp_precs.set(p);
//        }
        while ( !actions_pending_bwd().empty() ) {
            const Action* a = actions_pending_bwd().front();
            const Fluent* p = fluents_pending_bwd().front();
            actions_pending_bwd().pop();
            fluents_pending_bwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
			bool continue_flag=true;
            /** chao edit
            *
           */

//            if ( a->requires( p->index() ) ){
//                for (unsigned p: a->del_vec()){
//                    m_rp_precs.unset(p);
//                }
//            }

            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect
//                for(auto q : a->prec_vec())
//                {
//                    if (std::find(unachived_goal.begin(), unachived_goal.end(), q) != unachived_goal.end()){
//                        continue_flag= false;
//                        break;
//                    }
//                }
//                if (!continue_flag){
//                    continue;
//                }
                for ( auto q : a->prec_vec() )
                    m_rp_precs_bwd.set(q);

                if ( !extract_best_supporters_for_bwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }
            float min_cond_h = infty;
            //unsigned best_eff_index = no_such_index;
            Fluent_Vec tmp_cond;
            for ( unsigned k = 0; k < a->ceff_vec().size(); k++ ) {
                if ( !a->ceff_vec()[k]->asserts( p->index() ) ) continue;
                float h_cond;

                /** check later
                 *
                 * @param n
                 * @param h_val
                 */
                m_base_heuristic_bwd.eval_bwd( a->ceff_vec()[k]->prec_vec(), h_cond );
                if ( h_cond < min_cond_h ) {
                    min_cond_h = h_cond;
                    //best_eff_index = k;
                    tmp_cond = a->ceff_vec()[k]->prec_vec();
                    for ( auto p : a->prec_vec() )
                        tmp_cond.push_back( p );
                }
            }
            //assert( best_eff_index != no_such_index );
            for ( auto q : tmp_cond )
                m_rp_precs_bwd.set(q);
            if ( !extract_best_supporters_for_bwd( tmp_cond, relaxed_plan ) )
            {
                h_val = infty;
                assert( false );
                return;
            }
        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        if(!m_ignore_rp_h_value_bwd)
            h_val = 0.0f;


        for ( unsigned k = 0; k < G.size(); k++ )
            //if ( !init_fluents().isset( G[k] ) )
            m_rp_precs_bwd.set(G[k]);

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_bwd)
                h_val += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }

        std::vector< aptk::Action_Idx > app_set;
        m_strips_model_bwd.applicable_actions_v2( s, app_set );

        for (unsigned i = 0; i < app_set.size(); ++i) {
            //Successor_Generator::Iterator it( s, m_strips_model.successor_generator().nodes() );
            //int a = it.first();
            //while ( a != -1 ) {
            bool is_helpful = false;
            const Action& act = *(m_strips_model_bwd.actions()[app_set[i]]);
            for ( Fluent_Vec::const_iterator it2 = act.add_vec().begin();
                  it2 != act.add_vec().end(); it2++ )
                if ( m_rp_precs_bwd.isset( *it2 ) ) {
                    pref_ops.push_back( act.index() );
                    is_helpful = true;
                    //std::cout << "\t PO: " << m_strips_model.actions()[ act.index() ]->signature() << std::endl;
                    //Uncomment if just 1 pref op is preferred
                    if ( one_HA_per_fluent_bwd() )
                        m_rp_precs_bwd.unset(*it2);
                    break;
                }
            if ( is_helpful ) continue;
            // Check conditional effects
            for ( auto ceff : act.ceff_vec() ) {
                if ( !s.entails( ceff->prec_vec() ) ) continue;
                for ( auto p : ceff->add_vec() ) {
                    if ( m_rp_precs_bwd.isset( p ) ) {
                        pref_ops.push_back( act.index() );
                        is_helpful = true;
                        if ( one_HA_per_fluent_bwd() )
                            m_rp_precs_bwd.unset( p );
                        break;
                    }
                }
                if ( is_helpful ) break;
            }

            //a = it.next();
        }

    }
    /** chao edit
     * with action check
     */

    virtual void
    compute_edit_fwd( const State& s, float& h_val, std::vector<Action_Idx>& pref_ops, Fluent_Vec unachived_goal, std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {
        h_val = 0.0;
        // 0. Initialize data structures
        actions_seen_fwd().reset();
        init_fluents_fwd().reset();

//        m_base_heuristic.eval_edit( s, h_val );
//        if ( h_val == infty  )
//            return;

        /** chao test
         * original from current to init
         * new from init to current
         */
//        for ( unsigned k = 0; k < s.fluent_vec().size(); k++ )
//            init_fluents().set(s.fluent_vec()[k] );
//        while ( !actions_pending().empty() )
//            actions_pending().pop();
//        std::vector<const Action*> relaxed_plan;
//        const Fluent_Vec& G = this->m_strips_model.init();
//
//        for ( unsigned k = 0; k < G.size(); k++ ) {
//
//            if ( init_fluents().isset( G[k] ) ) continue;


//
        for ( unsigned k = 0; k < m_strips_model_fwd.init().size(); k++ )
            init_fluents_fwd().set(m_strips_model_fwd.init()[k] );

        while ( !actions_pending_fwd().empty() )
            actions_pending_fwd().pop();

        std::vector<const Action*> relaxed_plan;
//       change the m_strips_model.goal() to init()
        const Fluent_Vec& G = s.fluent_vec();
//        const Fluent_Vec& G = goals ? *goals : m_strips_model.init();

        // 1. Add to the pending queue best supporters for goal fluents

        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_fwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_fwd.get_best_supporter_fwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
//                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_fwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_fwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_fwd().push( m_strips_model_fwd.fluents()[G[k]] );
            actions_pending_fwd().push( sup );
            actions_seen_fwd().set( sup->index() );
            relaxed_plan.push_back( sup );

        }

        m_rp_precs_fwd.reset();
//        for (unsigned p:m_strips_model.fluents()){
//            m_rp_precs.set(p);
//        }
        while ( !actions_pending_fwd().empty() ) {
            const Action* a = actions_pending_fwd().front();
            const Fluent* p = fluents_pending_fwd().front();
            actions_pending_fwd().pop();
            fluents_pending_fwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
            bool continue_flag=true;
            /** chao edit
            *
           */

//            if ( a->requires( p->index() ) ){
//                for (unsigned p: a->del_vec()){
//                    m_rp_precs.unset(p);
//                }
//            }

            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect
//                for(auto q : a->prec_vec())
//                {
//                    if (std::find(unachived_goal.begin(), unachived_goal.end(), q) != unachived_goal.end()){
//                        continue_flag= false;
//                        break;
//                    }
//                }
//                if (!continue_flag){
//                    continue;
//                }
                for ( auto q : a->prec_vec() )
                    m_rp_precs_fwd.set(q);

                if ( !extract_best_supporters_for_fwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }
            float min_cond_h = infty;
            //unsigned best_eff_index = no_such_index;
            Fluent_Vec tmp_cond;
            for ( unsigned k = 0; k < a->ceff_vec().size(); k++ ) {
                if ( !a->ceff_vec()[k]->asserts( p->index() ) ) continue;
                float h_cond;

                /** check later
                 *
                 * @param n
                 * @param h_val
                 */
                m_base_heuristic_fwd.eval_fwd( a->ceff_vec()[k]->prec_vec(), h_cond );
                if ( h_cond < min_cond_h ) {
                    min_cond_h = h_cond;
                    //best_eff_index = k;
                    tmp_cond = a->ceff_vec()[k]->prec_vec();
                    for ( auto p : a->prec_vec() )
                        tmp_cond.push_back( p );
                }
            }
            //assert( best_eff_index != no_such_index );
            for ( auto q : tmp_cond )
                m_rp_precs_fwd.set(q);
            if ( !extract_best_supporters_for_fwd( tmp_cond, relaxed_plan ) )
            {
                h_val = infty;
                assert( false );
                return;
            }
        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        if(!m_ignore_rp_h_value_fwd)
            h_val = 0.0f;


        for ( unsigned k = 0; k < G.size(); k++ )
            //if ( !init_fluents().isset( G[k] ) )
            m_rp_precs_fwd.set(G[k]);

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_fwd)
                h_val += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }

        std::vector< aptk::Action_Idx > app_set;
        m_strips_model_fwd.applicable_actions_v2( s, app_set );

        for (unsigned i = 0; i < app_set.size(); ++i) {
            //Successor_Generator::Iterator it( s, m_strips_model.successor_generator().nodes() );
            //int a = it.first();
            //while ( a != -1 ) {
            bool is_helpful = false;
            const Action& act = *(m_strips_model_fwd.actions()[app_set[i]]);
            for ( Fluent_Vec::const_iterator it2 = act.add_vec().begin();
                  it2 != act.add_vec().end(); it2++ )
                if ( m_rp_precs_fwd.isset( *it2 ) ) {
                    pref_ops.push_back( act.index() );
                    is_helpful = true;
                    //std::cout << "\t PO: " << m_strips_model.actions()[ act.index() ]->signature() << std::endl;
                    //Uncomment if just 1 pref op is preferred
                    if ( one_HA_per_fluent_fwd() )
                        m_rp_precs_fwd.unset(*it2);
                    break;
                }
            if ( is_helpful ) continue;
            // Check conditional effects
            for ( auto ceff : act.ceff_vec() ) {
                if ( !s.entails( ceff->prec_vec() ) ) continue;
                for ( auto p : ceff->add_vec() ) {
                    if ( m_rp_precs_fwd.isset( p ) ) {
                        pref_ops.push_back( act.index() );
                        is_helpful = true;
                        if ( one_HA_per_fluent_fwd() )
                            m_rp_precs_fwd.unset( p );
                        break;
                    }
                }
                if ( is_helpful ) break;
            }

            //a = it.next();
        }

    }


    virtual void
    compute_action_edit_fwd( const State& s, float& h_val, std::vector<Action_Idx>& app_set, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {
        h_val = 0.0;
        // 0. Initialize data structures
        actions_seen_fwd().reset();
        init_fluents_fwd().reset();


        for ( unsigned k = 0; k < m_strips_model_fwd.init().size(); k++ )
            init_fluents_fwd().set(m_strips_model_fwd.init()[k] );

        while ( !actions_pending_fwd().empty() )
            actions_pending_fwd().pop();

        std::vector<const Action*> relaxed_plan;
//       change the m_strips_model.goal() to init()
        const Fluent_Vec& G = s.fluent_vec();
//        const Fluent_Vec& G = goals ? *goals : m_strips_model.init();

        // 1. Add to the pending queue best supporters for goal fluents

        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_fwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_fwd.get_best_supporter_fwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
//                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_fwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_fwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_fwd().push( m_strips_model_fwd.fluents()[G[k]] );
            actions_pending_fwd().push( sup );
            actions_seen_fwd().set( sup->index() );
            relaxed_plan.push_back( sup );

        }

        while ( !actions_pending_fwd().empty() ) {
            const Action* a = actions_pending_fwd().front();
            const Fluent* p = fluents_pending_fwd().front();
            actions_pending_fwd().pop();
            fluents_pending_fwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
            bool continue_flag=true;

            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect

                if ( !extract_best_supporters_for_fwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }
        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        if(!m_ignore_rp_h_value_fwd)
            h_val = 0.0f;

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_fwd)
                h_val += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }

        for (unsigned i = 0; i < app_set.size(); ++i) {
            const Action& act = *(m_strips_model_fwd.actions()[app_set[i]]);
            if (actions_seen_fwd().isset( act.index()) ) {
                pref_ops.push_back( act.index() );
            }
        }

    }


    virtual void
    compute_action_edit_bwd( const State& s, float& h_val, std::vector<Action_Idx>& app_set, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {
        h_val = 0.0;
        // 0. Initialize data structures
        actions_seen_bwd().reset();
        init_fluents_bwd().reset();


        for ( unsigned k = 0; k < m_strips_model_bwd.init().size(); k++ )
            init_fluents_bwd().set(m_strips_model_bwd.init()[k] );

        while ( !actions_pending_bwd().empty() )
            actions_pending_bwd().pop();

        std::vector<const Action*> relaxed_plan;
//       change the m_strips_model.goal() to init()
        const Fluent_Vec& G = s.fluent_vec();
//        const Fluent_Vec& G = goals ? *goals : m_strips_model.init();

        // 1. Add to the pending queue best supporters for goal fluents

        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_bwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_bwd.get_best_supporter_bwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
//                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_bwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_bwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_bwd().push( m_strips_model_bwd.fluents()[G[k]] );
            actions_pending_bwd().push( sup );
            actions_seen_bwd().set( sup->index() );
            relaxed_plan.push_back( sup );

        }

        while ( !actions_pending_bwd().empty() ) {
            const Action* a = actions_pending_bwd().front();
            const Fluent* p = fluents_pending_bwd().front();
            actions_pending_bwd().pop();
            fluents_pending_bwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
            bool continue_flag=true;

            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect

                if ( !extract_best_supporters_for_bwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }
        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        if(!m_ignore_rp_h_value_bwd)
            h_val = 0.0f;

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_bwd)
                h_val += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }

        for (unsigned i = 0; i < app_set.size(); ++i) {
            const Action& act = *(m_strips_model_bwd.actions()[app_set[i]]);
            if (actions_seen_bwd().isset( act.index()) ) {
                pref_ops.push_back( act.index() );
            }
        }

    }




    /** chao edit
     * with action check
     */

    virtual void
    compute_action_bwd( const State& s, const State& init, float& h_val,  std::vector<Action_Idx>& app_set, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {



        /** check later
         *
         * @param n
         * @param h_val
         */
        m_base_heuristic_bwd.eval_bwd( init, h_val );
        if ( h_val == infty  )
            return;


        // 0. Initialize data structures
        actions_seen_bwd().reset();
        init_fluents_bwd().reset();

        for ( unsigned k = 0; k < init.fluent_vec().size(); k++ )
            init_fluents_bwd().set( init.fluent_vec()[k] );

        while ( !actions_pending_bwd().empty() )
            actions_pending_bwd().pop();

        std::vector<const Action*> relaxed_plan;
//       change the m_strips_model.goal() to init()
        const Fluent_Vec& G = goals ? *goals : m_strips_model_bwd.goal();
//        const Fluent_Vec& G = goals ? *goals : m_strips_model.init();

        // 1. Add to the pending queue best supporters for goal fluents
        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_bwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_bwd.get_best_supporter_bwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_bwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_bwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_bwd().push( m_strips_model_bwd.fluents()[G[k]] );
            actions_pending_bwd().push( sup );
            actions_seen_bwd().set( sup->index() );
            relaxed_plan.push_back( sup );

        }

        while ( !actions_pending_bwd().empty() ) {
            const Action* a = actions_pending_bwd().front();
            const Fluent* p = fluents_pending_bwd().front();
            actions_pending_bwd().pop();
            fluents_pending_bwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect
                if ( !extract_best_supporters_for_bwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }

            //assert( best_eff_index != no_such_index );

        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        if(!m_ignore_rp_h_value_bwd)
            h_val = 0.0f;

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_bwd)
                h_val += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }


        for (unsigned i = 0; i < app_set.size(); ++i) {
            //Successor_Generator::Iterator it( s, m_strips_model.successor_generator().nodes() );
            //int a = it.first();
            //while ( a != -1 ) {
            const Action& act = *(m_strips_model_bwd.actions()[app_set[i]]);
            if (actions_seen_bwd().isset( act.index()) ) {
                    pref_ops.push_back( act.index() );
                }
        }

    }
    virtual void
    compute_action_fwd( const State& s, const State& init, float& h_val,  std::vector<Action_Idx>& app_set, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {



        /** check later
         *
         * @param n
         * @param h_val
         */
        m_base_heuristic_fwd.eval_fwd( init, h_val );
        if ( h_val == infty  )
            return;


        // 0. Initialize data structures
        actions_seen_fwd().reset();
        init_fluents_fwd().reset();

        for ( unsigned k = 0; k < init.fluent_vec().size(); k++ )
            init_fluents_fwd().set( init.fluent_vec()[k] );

        while ( !actions_pending_fwd().empty() )
            actions_pending_fwd().pop();

        std::vector<const Action*> relaxed_plan;
//       change the m_strips_model.goal() to init()
        const Fluent_Vec& G = goals ? *goals : m_strips_model_fwd.goal();
//        const Fluent_Vec& G = goals ? *goals : m_strips_model.init();

        // 1. Add to the pending queue best supporters for goal fluents
        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_fwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_fwd.get_best_supporter_fwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_fwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_fwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_fwd().push( m_strips_model_fwd.fluents()[G[k]] );
            actions_pending_fwd().push( sup );
            actions_seen_fwd().set( sup->index() );
            relaxed_plan.push_back( sup );

        }

        while ( !actions_pending_fwd().empty() ) {
            const Action* a = actions_pending_fwd().front();
            const Fluent* p = fluents_pending_fwd().front();
            actions_pending_fwd().pop();
            fluents_pending_fwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect
                if ( !extract_best_supporters_for_fwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }

            //assert( best_eff_index != no_such_index );

        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        if(!m_ignore_rp_h_value_fwd)
            h_val = 0.0f;

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_fwd)
                h_val += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }


        for (unsigned i = 0; i < app_set.size(); ++i) {
            //Successor_Generator::Iterator it( s, m_strips_model.successor_generator().nodes() );
            //int a = it.first();
            //while ( a != -1 ) {
            const Action& act = *(m_strips_model_fwd.actions()[app_set[i]]);
            if (actions_seen_fwd().isset( act.index()) ) {
                pref_ops.push_back( act.index() );
            }
        }

    }


    virtual void
    compute_fwd( const State& s, float& h_val, std::vector<Action_Idx>& pref_ops, State* goal= nullptr,std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {

        m_base_heuristic_fwd.eval_fwd( s, h_val,goal);
        if ( h_val == infty  )
            return;




        // 0. Initialize data structures
        actions_seen_fwd().reset();
        init_fluents_fwd().reset();

        for ( unsigned k = 0; k < s.fluent_vec().size(); k++ )
            init_fluents_fwd().set( s.fluent_vec()[k] );

        while ( !actions_pending_fwd().empty() )
            actions_pending_fwd().pop();

        if (goal!= nullptr){
            goals=&goal->fluent_vec();
        }

        std::vector<const Action*> relaxed_plan;
        const Fluent_Vec& G = goals ? *goals : m_strips_model_fwd.goal();

        // 1. Add to the pending queue best supporters for goal fluents
        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_fwd().isset( G[k] ) ) continue;
            const unsigned act_idx = m_base_heuristic_fwd.get_best_supporter_fwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_fwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_fwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_fwd().push( m_strips_model_fwd.fluents()[G[k]] );
            actions_pending_fwd().push( sup );
            actions_seen_fwd().set( sup->index() );
            relaxed_plan.push_back( sup );

        }

        m_rp_precs_fwd.reset();
        while ( !actions_pending_fwd().empty() ) {
            const Action* a = actions_pending_fwd().front();
            const Fluent* p = fluents_pending_fwd().front();
            actions_pending_fwd().pop();
            fluents_pending_fwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect
                for ( auto q : a->prec_vec() )
                    m_rp_precs_fwd.set(q);
                if ( !extract_best_supporters_for_fwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }
            float min_cond_h = infty;
            //unsigned best_eff_index = no_such_index;
            Fluent_Vec tmp_cond;
            for ( unsigned k = 0; k < a->ceff_vec().size(); k++ ) {
                if ( !a->ceff_vec()[k]->asserts( p->index() ) ) continue;
                float h_cond;
                m_base_heuristic_fwd.eval_fwd( a->ceff_vec()[k]->prec_vec(), h_cond );
                if ( h_cond < min_cond_h ) {
                    min_cond_h = h_cond;
                    //best_eff_index = k;
                    tmp_cond = a->ceff_vec()[k]->prec_vec();
                    for ( auto p : a->prec_vec() )
                        tmp_cond.push_back( p );
                }
            }
            //assert( best_eff_index != no_such_index );
            for ( auto q : tmp_cond )
                m_rp_precs_fwd.set(q);
            if ( !extract_best_supporters_for_fwd( tmp_cond, relaxed_plan ) )
            {
                h_val = infty;
                assert( false );
                return;
            }
        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        if(!m_ignore_rp_h_value_fwd)
            h_val = 0.0f;


        for ( unsigned k = 0; k < G.size(); k++ )
            //if ( !init_fluents().isset( G[k] ) )
            m_rp_precs_fwd.set(G[k]);

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_fwd)
                h_val += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }

        std::vector< aptk::Action_Idx > app_set;
        m_strips_model_fwd.applicable_actions_v2( s, app_set );

        for (unsigned i = 0; i < app_set.size(); ++i) {
            //Successor_Generator::Iterator it( s, m_strips_model.successor_generator().nodes() );
            //int a = it.first();
            //while ( a != -1 ) {
            bool is_helpful = false;
            const Action& act = *(m_strips_model_fwd.actions()[app_set[i]]);
            for ( Fluent_Vec::const_iterator it2 = act.add_vec().begin();
                  it2 != act.add_vec().end(); it2++ )
                if ( m_rp_precs_fwd.isset( *it2 ) ) {
                    pref_ops.push_back( act.index() );
                    is_helpful = true;
                    //std::cout << "\t PO: " << m_strips_model.actions()[ act.index() ]->signature() << std::endl;
                    //Uncomment if just 1 pref op is preferred
                    if ( one_HA_per_fluent_fwd() )
                        m_rp_precs_fwd.unset(*it2);
                    break;
                }
            if ( is_helpful ) continue;
            // Check conditional effects
            for ( auto ceff : act.ceff_vec() ) {
                if ( !s.entails( ceff->prec_vec() ) ) continue;
                for ( auto p : ceff->add_vec() ) {
                    if ( m_rp_precs_fwd.isset( p ) ) {
                        pref_ops.push_back( act.index() );
                        is_helpful = true;
                        if ( one_HA_per_fluent_fwd() )
                            m_rp_precs_fwd.unset( p );
                        break;
                    }
                }
                if ( is_helpful ) break;
            }

            //a = it.next();
        }

    }

    virtual void
    compute_bwd( const State& s, float& h_val, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {
#if 0
        /** original
          *
         */
//		m_base_heuristic.eval( s, h_val );
//		if ( h_val == infty  )
//			return;


        /** chao edit
         *
         */
#endif
        /** check later
         *
         * @param n
         * @param h_val
         */

        m_base_heuristic_bwd.eval_bwd( s, h_val);
        if ( h_val == infty  )
            return;


        // 0. Initialize data structures
        actions_seen_bwd().reset();
        init_fluents_bwd().reset();

        for ( unsigned k = 0; k < s.fluent_vec().size(); k++ )
            init_fluents_bwd().set( s.fluent_vec()[k] );

        while ( !actions_pending_bwd().empty() )
            actions_pending_bwd().pop();

        std::vector<const Action*> relaxed_plan;

//       change the m_strips_model.goal() to init()
        const Fluent_Vec& G = goals ? *goals : m_strips_model_bwd.goal();
//        const Fluent_Vec& G = goals ? *goals : m_strips_model.init();

        // 1. Add to the pending queue best supporters for goal fluents
        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_bwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_bwd.get_best_supporter_bwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_bwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_bwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_bwd().push( m_strips_model_bwd.fluents()[G[k]] );
            actions_pending_bwd().push( sup );
            actions_seen_bwd().set( sup->index() );

            relaxed_plan.push_back( sup );



        }

        m_rp_precs_bwd.reset();
        while ( !actions_pending_bwd().empty() ) {
            const Action* a = actions_pending_bwd().front();
            const Fluent* p = fluents_pending_bwd().front();
            actions_pending_bwd().pop();
            fluents_pending_bwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect
                for ( auto q : a->prec_vec() )
                    m_rp_precs_bwd.set(q);
                if ( !extract_best_supporters_for_bwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }
            float min_cond_h = infty;
            //unsigned best_eff_index = no_such_index;
            Fluent_Vec tmp_cond;
            for ( unsigned k = 0; k < a->ceff_vec().size(); k++ ) {
                if ( !a->ceff_vec()[k]->asserts( p->index() ) ) continue;
                float h_cond;
                m_base_heuristic_bwd.eval_bwd( a->ceff_vec()[k]->prec_vec(), h_cond );
                if ( h_cond < min_cond_h ) {
                    min_cond_h = h_cond;
                    //best_eff_index = k;
                    tmp_cond = a->ceff_vec()[k]->prec_vec();
                    for ( auto p : a->prec_vec() )
                        tmp_cond.push_back( p );
                }
            }
            //assert( best_eff_index != no_such_index );
            for ( auto q : tmp_cond )
                m_rp_precs_bwd.set(q);
            if ( !extract_best_supporters_for_bwd( tmp_cond, relaxed_plan ) )
            {
                h_val = infty;
                assert( false );
                return;
            }
        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        if(!m_ignore_rp_h_value_bwd)
            h_val = 0.0f;


        for ( unsigned k = 0; k < G.size(); k++ )
            //if ( !init_fluents().isset( G[k] ) )
            m_rp_precs_bwd.set(G[k]);

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_bwd)
                h_val += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }

        std::vector< aptk::Action_Idx > app_set;
        m_strips_model_bwd.applicable_actions_v2( s, app_set );

        for (unsigned i = 0; i < app_set.size(); ++i) {
            //Successor_Generator::Iterator it( s, m_strips_model.successor_generator().nodes() );
            //int a = it.first();
            //while ( a != -1 ) {
            bool is_helpful = false;
            const Action& act = *(m_strips_model_bwd.actions()[app_set[i]]);
            for ( Fluent_Vec::const_iterator it2 = act.add_vec().begin();
                  it2 != act.add_vec().end(); it2++ )
                if ( m_rp_precs_bwd.isset( *it2 ) ) {
                    pref_ops.push_back( act.index() );
                    is_helpful = true;
                    //std::cout << "\t PO: " << m_strips_model.actions()[ act.index() ]->signature() << std::endl;
                    //Uncomment if just 1 pref op is preferred
                    if ( one_HA_per_fluent_bwd() )
                        m_rp_precs_bwd.unset(*it2);
                    break;
                }
            if ( is_helpful ) continue;
            // Check conditional effects
            for ( auto ceff : act.ceff_vec() ) {
                if ( !s.entails( ceff->prec_vec() ) ) continue;
                for ( auto p : ceff->add_vec() ) {
                    if ( m_rp_precs_bwd.isset( p ) ) {
                        pref_ops.push_back( act.index() );
                        is_helpful = true;
                        if ( one_HA_per_fluent_bwd() )
                            m_rp_precs_bwd.unset( p );
                        break;
                    }
                }
                if ( is_helpful ) break;
            }

            //a = it.next();
        }

    }

	virtual void 
	compute_bwd( const State& s, float& h_val, float& h_val_rp, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {


        /** check later
         *
         * @param n
         * @param h_val
         */
		m_base_heuristic_bwd.eval_bwd( s, h_val );
		if ( h_val == infty )
			return;

		// 0. Initialize data structures
		actions_seen_bwd().reset();
		init_fluents_bwd().reset();

		for ( unsigned k = 0; k < s.fluent_vec().size(); k++ )
			init_fluents_bwd().set( s.fluent_vec()[k] );
	
		while ( !actions_pending_bwd().empty() )
			actions_pending_bwd().pop();
	
		std::vector<const Action*> relaxed_plan;
		const Fluent_Vec& G = goals ? *goals : m_strips_model_bwd.goal();
	
		// 1. Add to the pending queue best supporters for goal fluents
		for ( unsigned k = 0; k < G.size(); k++ ) {
	
			if ( init_fluents_bwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
			const unsigned act_idx = m_base_heuristic_bwd.get_best_supporter_bwd( G[k] ).act_idx;
			if ( act_idx == no_such_index ) // No best supporter for fluent
			{
				std::cerr << "No best supporter found for goal fluent ";
				std::cerr << m_strips_model_bwd.fluents()[G[k]]->signature() << std::endl;
				return;
			}
			const Action* sup = m_strips_model_bwd.actions()[ act_idx ];
			#ifdef DEBUG_RP_HEURISTIC
			std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
			#endif
			fluents_pending_bwd().push( m_strips_model_bwd.fluents()[G[k]] );
			actions_pending_bwd().push( sup );
			actions_seen_bwd().set( sup->index() );
			relaxed_plan.push_back( sup );

		}	
	
		m_rp_precs_bwd.reset();
		while ( !actions_pending_bwd().empty() ) {
			const Action* a = actions_pending_bwd().front();
			const Fluent* p = fluents_pending_bwd().front();
			actions_pending_bwd().pop();
			fluents_pending_bwd().pop();
			#ifdef DEBUG_RP_HEURISTIC
			std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
			#endif
			if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect
				for ( auto q : a->prec_vec() )
					m_rp_precs_bwd.set(q);
				if ( !extract_best_supporters_for_bwd( a->prec_vec(), relaxed_plan ) ) {
					h_val = infty;
					assert( false );
					return;
				}
				continue;
			}
			float min_cond_h = infty;
			//unsigned best_eff_index = no_such_index;
			Fluent_Vec tmp_cond;
			for ( unsigned k = 0; k < a->ceff_vec().size(); k++ ) {
				if ( !a->ceff_vec()[k]->asserts( p->index() ) ) continue;	
				float h_cond;

                /** check later
                 *
                 * @param n
                 * @param h_val
                 */
				m_base_heuristic_bwd.eval_bwd( a->ceff_vec()[k]->prec_vec(), h_cond );
				if ( h_cond < min_cond_h ) {
					min_cond_h = h_cond;
					//best_eff_index = k;
					tmp_cond = a->ceff_vec()[k]->prec_vec();
					for ( auto p : a->prec_vec() )
						tmp_cond.push_back( p );
				}
			}
			//assert( best_eff_index != no_such_index );
			for ( auto q : tmp_cond )
				m_rp_precs_bwd.set(q);
			if ( !extract_best_supporters_for_bwd( tmp_cond, relaxed_plan ) )
			{
				h_val = infty;
				assert( false );
				return;
			}			
		}

		if(copy_rel_plan) 
			for(unsigned i = 0; i < relaxed_plan.size(); i++)
				copy_rel_plan->push_back( relaxed_plan[i]->index() );

		//if(!m_ignore_rp_h_value)
		h_val_rp = 0.0f;
	
		
		for ( unsigned k = 0; k < G.size(); k++ ) 
			//if ( !init_fluents().isset( G[k] ) )
				m_rp_precs_bwd.set(G[k]);
	
		#ifdef DEBUG_RP_HEURISTIC	
		std::cout << "\nRel Plan: ";
		#endif
		for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
			if(!m_ignore_rp_h_value_bwd)
				h_val_rp += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
			//const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
			#ifdef DEBUG_RP_HEURISTIC
			std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
			#endif
		}
	
		std::vector< aptk::Action_Idx > app_set;
		m_strips_model_bwd.applicable_actions_v2( s, app_set );
		
		for (unsigned i = 0; i < app_set.size(); ++i) {
		//Successor_Generator::Iterator it( s, m_strips_model.successor_generator().nodes() );
		//int a = it.first();
		//while ( a != -1 ) {
			bool is_helpful = false;
			const Action& act = *(m_strips_model_bwd.actions()[app_set[i]]);
			for ( Fluent_Vec::const_iterator it2 = act.add_vec().begin();
				it2 != act.add_vec().end(); it2++ )
				if ( m_rp_precs_bwd.isset( *it2 ) ) {
					pref_ops.push_back( act.index() );
					is_helpful = true;
					//std::cout << "\t PO: " << m_strips_model.actions()[ act.index() ]->signature() << std::endl;
					//Uncomment if just 1 pref op is preferred
					if ( one_HA_per_fluent_bwd() )
						m_rp_precs_bwd.unset(*it2);
					break;
				}
			if ( is_helpful ) continue;	
			// Check conditional effects
			for ( auto ceff : act.ceff_vec() ) {
				if ( !s.entails( ceff->prec_vec() ) ) continue;
				for ( auto p : ceff->add_vec() ) {
					if ( m_rp_precs_bwd.isset( p ) ) {
						pref_ops.push_back( act.index() );
						is_helpful = true;
						if ( one_HA_per_fluent_bwd() )
							m_rp_precs_bwd.unset( p );
						break;
					}
				}
				if ( is_helpful ) break;
			}

			//a = it.next();
		}
	
	}
    virtual void
    compute_fwd( const State& s, float& h_val, float& h_val_rp, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>* copy_rel_plan = NULL, Fluent_Vec* goals = NULL ) {


        /** check later
         *
         * @param n
         * @param h_val
         */
        m_base_heuristic_fwd.eval_fwd( s, h_val );
        if ( h_val == infty )
            return;

        // 0. Initialize data structures
        actions_seen_fwd().reset();
        init_fluents_fwd().reset();

        for ( unsigned k = 0; k < s.fluent_vec().size(); k++ )
            init_fluents_fwd().set( s.fluent_vec()[k] );

        while ( !actions_pending_fwd().empty() )
            actions_pending_fwd().pop();

        std::vector<const Action*> relaxed_plan;
        const Fluent_Vec& G = goals ? *goals : m_strips_model_fwd.goal();

        // 1. Add to the pending queue best supporters for goal fluents
        for ( unsigned k = 0; k < G.size(); k++ ) {

            if ( init_fluents_fwd().isset( G[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_fwd.get_best_supporter_fwd( G[k] ).act_idx;
            if ( act_idx == no_such_index ) // No best supporter for fluent
            {
                std::cerr << "No best supporter found for goal fluent ";
                std::cerr << m_strips_model_fwd.fluents()[G[k]]->signature() << std::endl;
                return;
            }
            const Action* sup = m_strips_model_fwd.actions()[ act_idx ];
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Goal: " << m_strips_model.fluents()[G[k]]->signature() << std::endl;
			std::cout << "Value = " << m_base_heuristic.value( G[k] ) << std::endl;
			std::cout << "Best supporter: " << sup->signature() << std::endl;
#endif
            fluents_pending_fwd().push( m_strips_model_fwd.fluents()[G[k]] );
            actions_pending_fwd().push( sup );
            actions_seen_fwd().set( sup->index() );
            relaxed_plan.push_back( sup );

        }

        m_rp_precs_fwd.reset();
        while ( !actions_pending_fwd().empty() ) {
            const Action* a = actions_pending_fwd().front();
            const Fluent* p = fluents_pending_fwd().front();
            actions_pending_fwd().pop();
            fluents_pending_fwd().pop();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "Getting action: " << std::endl;
			a->print( m_strips_model, std::cout );
			std::cout << "into the relaxed plan" << std::endl;
#endif
            if ( a->asserts( p->index() ) ) { // fluent asserted by action main effect
                for ( auto q : a->prec_vec() )
                    m_rp_precs_fwd.set(q);
                if ( !extract_best_supporters_for_fwd( a->prec_vec(), relaxed_plan ) ) {
                    h_val = infty;
                    assert( false );
                    return;
                }
                continue;
            }
            float min_cond_h = infty;
            //unsigned best_eff_index = no_such_index;
            Fluent_Vec tmp_cond;
            for ( unsigned k = 0; k < a->ceff_vec().size(); k++ ) {
                if ( !a->ceff_vec()[k]->asserts( p->index() ) ) continue;
                float h_cond;

                /** check later
                 *
                 * @param n
                 * @param h_val
                 */
                m_base_heuristic_fwd.eval_fwd( a->ceff_vec()[k]->prec_vec(), h_cond );
                if ( h_cond < min_cond_h ) {
                    min_cond_h = h_cond;
                    //best_eff_index = k;
                    tmp_cond = a->ceff_vec()[k]->prec_vec();
                    for ( auto p : a->prec_vec() )
                        tmp_cond.push_back( p );
                }
            }
            //assert( best_eff_index != no_such_index );
            for ( auto q : tmp_cond )
                m_rp_precs_fwd.set(q);
            if ( !extract_best_supporters_for_fwd( tmp_cond, relaxed_plan ) )
            {
                h_val = infty;
                assert( false );
                return;
            }
        }

        if(copy_rel_plan)
            for(unsigned i = 0; i < relaxed_plan.size(); i++)
                copy_rel_plan->push_back( relaxed_plan[i]->index() );

        //if(!m_ignore_rp_h_value)
        h_val_rp = 0.0f;


        for ( unsigned k = 0; k < G.size(); k++ )
            //if ( !init_fluents().isset( G[k] ) )
            m_rp_precs_fwd.set(G[k]);

#ifdef DEBUG_RP_HEURISTIC
        std::cout << "\nRel Plan: ";
#endif
        for ( unsigned k = 0; k < relaxed_plan.size(); k++ ) {
            if(!m_ignore_rp_h_value_fwd)
                h_val_rp += ( cost_opt == RP_Cost_Function::Ignore_Costs ? 1.0f : relaxed_plan[k]->cost() );
            //const Fluent_Vec& precs = relaxed_plan[k]->prec_vec();
#ifdef DEBUG_RP_HEURISTIC
            std::cout << "\t "<< k <<": " << relaxed_plan[k]->signature() << std::endl;
#endif
        }

        std::vector< aptk::Action_Idx > app_set;
        m_strips_model_fwd.applicable_actions_v2( s, app_set );

        for (unsigned i = 0; i < app_set.size(); ++i) {
            //Successor_Generator::Iterator it( s, m_strips_model.successor_generator().nodes() );
            //int a = it.first();
            //while ( a != -1 ) {
            bool is_helpful = false;
            const Action& act = *(m_strips_model_fwd.actions()[app_set[i]]);
            for ( Fluent_Vec::const_iterator it2 = act.add_vec().begin();
                  it2 != act.add_vec().end(); it2++ )
                if ( m_rp_precs_fwd.isset( *it2 ) ) {
                    pref_ops.push_back( act.index() );
                    is_helpful = true;
                    //std::cout << "\t PO: " << m_strips_model.actions()[ act.index() ]->signature() << std::endl;
                    //Uncomment if just 1 pref op is preferred
                    if ( one_HA_per_fluent_fwd() )
                        m_rp_precs_fwd.unset(*it2);
                    break;
                }
            if ( is_helpful ) continue;
            // Check conditional effects
            for ( auto ceff : act.ceff_vec() ) {
                if ( !s.entails( ceff->prec_vec() ) ) continue;
                for ( auto p : ceff->add_vec() ) {
                    if ( m_rp_precs_fwd.isset( p ) ) {
                        pref_ops.push_back( act.index() );
                        is_helpful = true;
                        if ( one_HA_per_fluent_fwd() )
                            m_rp_precs_fwd.unset( p );
                        break;
                    }
                }
                if ( is_helpful ) break;
            }

            //a = it.next();
        }

    }

	void ignore_rp_h_value_bwd(bool b) {m_ignore_rp_h_value_bwd = b;}
    void ignore_rp_h_value_fwd(bool b) {m_ignore_rp_h_value_fwd = b;}

protected:

	Bit_Array&		actions_seen_bwd() { return m_act_seen_bwd; }
	Action_Queue&		actions_pending_bwd() { return m_pending_bwd; }
	Fluent_Queue&		fluents_pending_bwd() { return m_pending_fluents_bwd; }
	Bit_Array&		init_fluents_bwd() { return m_init_fluents_bwd; }

    Bit_Array&		actions_seen_fwd() { return m_act_seen_fwd; }
    Action_Queue&		actions_pending_fwd() { return m_pending_fwd; }
    Fluent_Queue&		fluents_pending_fwd() { return m_pending_fluents_fwd; }
    Bit_Array&		init_fluents_fwd() { return m_init_fluents_fwd; }

	bool	extract_best_supporters_for_bwd( const Fluent_Vec& C, std::vector<const Action*>& relaxed_plan ) {
		for ( unsigned k = 0; k < C.size(); k++ ) {
	
			if ( init_fluents_bwd().isset( C[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
			const unsigned act_idx = m_base_heuristic_bwd.get_best_supporter_bwd( C[k] ).act_idx;
			
			if ( act_idx == no_such_index )
			{
				std::cerr << "No best supporter found for fluent ";
				std::cerr << m_strips_model_bwd.fluents()[C[k]]->signature() << std::endl;

                /** check later
                 *
                 * @param n
                 * @param h_val
                 */
				std::cerr << "Value = " << m_base_heuristic_bwd.value_bwd( C[k] ) << std::endl;
				exit(1);
				return false;
			}
			const Action* sup = m_strips_model_bwd.actions()[ act_idx ];
			if ( actions_seen_bwd().isset( sup->index() ) ) continue;
			fluents_pending_bwd().push( m_strips_model_bwd.fluents()[C[k]] );
			actions_pending_bwd().push( sup );
			actions_seen_bwd().set( sup->index() );
			relaxed_plan.push_back( sup );
		} 
		return true;
	}
    bool	extract_best_supporters_for_fwd( const Fluent_Vec& C, std::vector<const Action*>& relaxed_plan ) {
        for ( unsigned k = 0; k < C.size(); k++ ) {

            if ( init_fluents_fwd().isset( C[k] ) ) continue;

            /** check later
             *
             * @param n
             * @param h_val
             */
            const unsigned act_idx = m_base_heuristic_fwd.get_best_supporter_fwd( C[k] ).act_idx;

            if ( act_idx == no_such_index )
            {
                std::cerr << "No best supporter found for fluent ";
                std::cerr << m_strips_model_fwd.fluents()[C[k]]->signature() << std::endl;

                /** check later
                 *
                 * @param n
                 * @param h_val
                 */
                std::cerr << "Value = " << m_base_heuristic_fwd.value_fwd( C[k] ) << std::endl;
                exit(1);
                return false;
            }
            const Action* sup = m_strips_model_fwd.actions()[ act_idx ];
            if ( actions_seen_fwd().isset( sup->index() ) ) continue;
            fluents_pending_fwd().push( m_strips_model_fwd.fluents()[C[k]] );
            actions_pending_fwd().push( sup );
            actions_seen_fwd().set( sup->index() );
            relaxed_plan.push_back( sup );
        }
        return true;
    }


protected:

	
//	Primary_Heuristic&		m_base_heuristic;
    Primary_Heuristic&		m_base_heuristic_bwd;
    Primary_Heuristic&		m_base_heuristic_fwd;

	//Bit_Array			m_act_seen;
    Bit_Array			m_act_seen_bwd;
    Bit_Array			m_act_seen_fwd;

	//Bit_Array			m_init_fluents;
    Bit_Array			m_init_fluents_bwd;
    Bit_Array			m_init_fluents_fwd;

	//Action_Queue			m_pending;
    Action_Queue			m_pending_bwd;
    Action_Queue			m_pending_fwd;


//	Fluent_Queue			m_pending_fluents;
    Fluent_Queue			m_pending_fluents_bwd;
    Fluent_Queue			m_pending_fluents_fwd;


	const STRIPS_Problem&		m_strips_model_bwd;
    const STRIPS_Problem&		m_strips_model_fwd;

	//Bit_Set				m_po_set;
    Bit_Set				m_po_set_bwd;
    Bit_Set				m_po_set_fwd;

	//Bit_Set				m_rp_precs;
    Bit_Set				m_rp_precs_bwd;
    Bit_Set				m_rp_precs_fwd;

	//bool                            m_ignore_rp_h_value;
    bool                            m_ignore_rp_h_value_bwd;
    bool                            m_ignore_rp_h_value_fwd;


	//bool				m_one_ha_per_fluent;
    bool				m_one_ha_per_fluent_bwd;
    bool				m_one_ha_per_fluent_fwd;
};

template < typename bwd_Search_Problem,typename  Fwd_Search_Problem, typename Primary_Heuristic, RP_Cost_Function cost_opt = RP_Cost_Function::Use_Costs >
class Relaxed_Plan_Heuristic : public Heuristic<State>
{
public:

	Relaxed_Plan_Heuristic( const bwd_Search_Problem& prob_bwd, const Fwd_Search_Problem& prob_fwd )
	: Heuristic<State>(prob_bwd,prob_fwd), m_base_heuristic(prob_bwd,prob_fwd),m_plan_extractor( prob_bwd.task(), m_base_heuristic, prob_fwd.task(),m_base_heuristic ) {

	}

	virtual ~Relaxed_Plan_Heuristic() {}

	template <typename Search_Node, typename Cost_Type>
        void eval_bwd( const Search_Node* n, Cost_Type& h_val, std::vector<Action_Idx>& pref_ops) {
		eval_bwd(n->state(), h_val, pref_ops);
	}

    template <typename Search_Node, typename Cost_Type>
    void eval_fwd( const Search_Node* n, Cost_Type& h_val, std::vector<Action_Idx>& pref_ops) {
        eval_fwd(n->state(), h_val, pref_ops);
    }

	
	template <typename Search_Node, typename Cost_Type>
        void eval_bwd( const Search_Node* n, Cost_Type& h_val ) {
		
		eval_bwd(n->state(),h_val);
	}
    template <typename Search_Node, typename Cost_Type>
    void eval_fwd( const Search_Node* n, Cost_Type& h_val ) {

        eval_fwd(n->state(),h_val);
    }



	template <typename Cost_Type>
	void eval_bwd( const State& s, Cost_Type& h_out ) {
		float h;
		std::vector<Action_Idx> po;
		eval_bwd( s, h, po);
		h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
		
	}
    template <typename Cost_Type>
    void eval_fwd( const State& s, Cost_Type& h_out ) {
        float h;
        std::vector<Action_Idx> po;
        eval_fwd( s, h, po);
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;

    }

	
	template <typename Cost_Type>
	void eval_bwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& pref_ops ) {
		float h;
		m_plan_extractor.compute_bwd( s, h, pref_ops );
		h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
	}
    template <typename Cost_Type>
    void eval_fwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& pref_ops ) {
        float h;
        m_plan_extractor.compute_fwd( s, h, pref_ops );
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
    }
	/** chao dit with action check
	 *
	 */
    template <typename Cost_Type>
    void eval_4h_bwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& app_set, std::vector<Action_Idx>& pref_ops ) {
        float h;
        m_plan_extractor.compute_action_edit_bwd( s, h, app_set, pref_ops );
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
    }
    template <typename Cost_Type>
    void eval_4h_fwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& app_set, std::vector<Action_Idx>& pref_ops ) {
        float h;
        m_plan_extractor.compute_action_edit_fwd( s, h, app_set, pref_ops );
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
    }



	
	template <typename Cost_Type>
	void eval_bwd( const State& s, Cost_Type& h_out, Cost_Type& h_out_rp, std::vector<Action_Idx>& pref_ops ) {
		float h, h_rp;		
		m_plan_extractor.compute_bwd( s, h, h_rp, pref_ops );
		h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
		h_out_rp = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h_rp;
		
	}
    template <typename Cost_Type>
    void eval_fwd( const State& s, Cost_Type& h_out, Cost_Type& h_out_rp, std::vector<Action_Idx>& pref_ops ) {
        float h, h_rp;
        m_plan_extractor.compute_fwd( s, h, h_rp, pref_ops );
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
        h_out_rp = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h_rp;

    }

	template <typename Cost_Type>
	void eval_bwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>& rel_plan ) {
		float h;
		m_plan_extractor.compute_bwd( s, h, pref_ops, &rel_plan );
		h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
	}
    template <typename Cost_Type>
    void eval_fwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>& rel_plan, State* goal=nullptr) {
        float h;
        m_plan_extractor.compute_fwd( s, h,pref_ops,goal, &rel_plan );
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
    }
    template <typename Cost_Type>
    /** chao edit
     * with action check
     */
    void eval_chao_bwd( const State& s, const State& init,Cost_Type& h_out, std::vector<Action_Idx>& app_set, std::vector<Action_Idx>& pref_ops ) {
        float h;
        m_plan_extractor.compute_action_bwd( s, init, h, app_set, pref_ops);
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
    }
    template <typename Cost_Type>
    void eval_chao_fwd( const State& s, const State& init,Cost_Type& h_out, std::vector<Action_Idx>& app_set, std::vector<Action_Idx>& pref_ops ) {
        float h;
        m_plan_extractor.compute_action_fwd( s, init, h, app_set, pref_ops);
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
    }
	/**
	 * chao edit
	 */
    template <typename Cost_Type>
    void eval_bwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>& rel_plan, bool flag, Fluent_Vec unachived_goal, State* goal= nullptr) {
        float h;
        m_plan_extractor.compute_edit_bwd( s, h, pref_ops, unachived_goal, &rel_plan, goal );
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
    }

  	template <typename Cost_Type>
	void eval_fwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& pref_ops, Fluent_Vec* goals ) {
		float h;
		m_plan_extractor.compute_fwd( s, h, pref_ops, NULL, goals );
		h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
	}		

    	template <typename Cost_Type>
	void eval_bwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>& rel_plan, Fluent_Vec* goals ) {
		float h;
		m_plan_extractor.compute_bwd( s, h, pref_ops, &rel_plan, goals );
		h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
	}
    template <typename Cost_Type>
    void eval_fwd( const State& s, Cost_Type& h_out, std::vector<Action_Idx>& pref_ops, std::vector<Action_Idx>& rel_plan, Fluent_Vec* goals ) {
        float h;
        m_plan_extractor.compute_fwd( s, h, pref_ops, &rel_plan, goals );
        h_out = h == infty ? std::numeric_limits<Cost_Type>::max() : (Cost_Type)h;
    }

    void ignore_rp_h_value_bwd(bool b) {m_plan_extractor.ignore_rp_h_value_bwd(b);}

	void set_one_HA_per_fluent_bwd( bool b ) { m_plan_extractor.set_one_HA_per_fluent_bwd(b); }
	
	bool is_relaxed_plan_relevant_bwd( unsigned p ) { return m_plan_extractor.is_relaxed_plan_relevant_bwd(p); }

	unsigned value_bwd( unsigned p ){ return m_base_heuristic.value_bwd(p); }


    void ignore_rp_h_value_fwd(bool b) {m_plan_extractor.ignore_rp_h_value_fwd(b);}

    void set_one_HA_per_fluent_fwd( bool b ) { m_plan_extractor.set_one_HA_per_fluent_fwd(b); }

    bool is_relaxed_plan_relevant_fwd( unsigned p ) { return m_plan_extractor.is_relaxed_plan_relevant_fwd(p); }

    unsigned value_fwd( unsigned p ){ return m_base_heuristic.value_fwd(p); }
protected:

	Primary_Heuristic					m_base_heuristic;
//    Primary_Heuristic					m_base_heuristic_bwd;
//    Primary_Heuristic					m_base_heuristic_fwd;

	Relaxed_Plan_Extractor< Primary_Heuristic, cost_opt >	m_plan_extractor;
//    Relaxed_Plan_Extractor< Primary_Heuristic, cost_opt >	m_plan_extractor_bwd;
//    Relaxed_Plan_Extractor< Primary_Heuristic, cost_opt >	m_plan_extractor_fwd;
};

}

}

#endif // rp_heuristic.hxx
