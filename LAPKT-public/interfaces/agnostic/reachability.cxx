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

#include <reachability.hxx>
#include <action.hxx>
#include "bwd_search_prob.hxx"

namespace aptk {

namespace agnostic {

Reachability_Test::Reachability_Test( const STRIPS_Problem& p )
	: m_problem( p )
{
	m_reachable_atoms.resize( m_problem.fluents().size() );
	m_reach_next.resize( m_problem.fluents().size() );
	m_action_mask.resize( m_problem.actions().size() );
	/** chao add
	 *
	 */
    m_reachable_negation_atoms.resize( m_problem.negation_fluents().size() );
    m_reach_negation_next.resize( m_problem.negation_fluents().size() );
    m_negation_action_mask.resize( m_problem.negation_actions().size() );



}

Reachability_Test::~Reachability_Test()
{

}

void	Reachability_Test::initialize( const Fluent_Vec& s)
{
	for ( unsigned i = 0; i < m_reachable_atoms.size(); i++ )
		m_reachable_atoms[i] = false;


	m_action_mask.reset();
	
	for ( unsigned i = 0; i < s.size(); i++ ) {

		#ifdef DEBUG
		std::cout << "Fluent " << m_problem.fluents()[s[i]]->signature() << " initially true" << std::endl;
		#endif
		m_reachable_atoms[ s[i] ] = true;

	}
}

/** chao edit
 *
 */

    void	Reachability_Test::initialize_negation( const Fluent_Vec& s)
    {
        for ( unsigned i = 0; i < m_reachable_negation_atoms.size(); i++ )
            m_reachable_negation_atoms[i] = false;


        m_negation_action_mask.reset();

        for ( unsigned i = 0; i < s.size(); i++ ) {

#ifdef DEBUG
            std::cout << "Fluent " << m_problem.fluents()[s[i]]->signature() << " initially true" << std::endl;
#endif
            m_reachable_negation_atoms[ s[i] ] = true;

        }
    }

    std::vector <std::string> split(const std::string &str, const std::string &pattern) {
        std::vector<std::string> res;
        if (str == "")
            return res;
        std::string strs = str + pattern;
        size_t pos = strs.find(pattern);

        while (pos != strs.npos) {
            std::string temp = strs.substr(0, pos);
            res.push_back(temp);

            strs = strs.substr(pos + 1, strs.size());
            pos = strs.find(pattern);
        }

        return res;
    }


    bool	Reachability_Test::apply_actions_original() {

        bool fixed_point = true;
        m_reach_next = m_reachable_atoms;

        for ( unsigned i = 0; i < m_problem.actions().size(); i++ )
        {
            if ( m_action_mask.isset(i) ) continue;

            const Action* a = m_problem.actions()[i];

            // Check if applicable
            const Fluent_Vec&	pi = a->prec_vec();
            bool		applicable = true;
            for ( unsigned j = 0; j < pi.size(); j++ )
                if ( !m_reachable_atoms[pi[j]] )
                {
                    applicable = false;
                    break;
                }

            if ( !applicable ) continue;

#ifdef DEBUG
            std::cout << "Applying " << a->signature() << std::endl;
#endif

            // Apply effects
            const Fluent_Vec& ai = a->add_vec();
            for ( unsigned j = 0; j < ai.size(); j++ ) {
                if ( !m_reachable_atoms[ai[j]] ) {
                    m_reach_next[ai[j]] = true;
                    fixed_point = false;
                }
            }

            bool all_ce_applied = true;

            for( unsigned j = 0; j < a->ceff_vec().size(); j++ ) {

                const Fluent_Vec&	pi = a->ceff_vec()[j]->prec_vec();
                bool			applicable = true;

                for ( unsigned k = 0; k < pi.size(); k++ )
                    if ( !m_reachable_atoms[pi[k]] ) {
                        applicable = false;
                        break;
                    }

                if ( !applicable ) {
                    all_ce_applied = false;
                    continue;
                }

                const Fluent_Vec&	ai = a->ceff_vec()[j]->add_vec();

                for ( unsigned k = 0; k < ai.size(); k++ ) {

                    if ( !m_reachable_atoms[ai[k]] ) {

                        m_reach_next[ai[k]] = true;
                        fixed_point = false;
                    }
                }

            }
            if ( all_ce_applied ) m_action_mask.set(i);
        }
        m_reachable_atoms = m_reach_next;
        return fixed_point;
    }

bool	Reachability_Test::apply_actions() {

	bool fixed_point = true;
	m_reach_next = m_reachable_atoms;

	for ( unsigned i = 0; i < m_problem.actions().size(); i++ )
	{
		if ( m_action_mask.isset(i) ) continue;

		const Action* a = m_problem.actions()[i];
		// Check if applicable
//		const Fluent_Vec&	pi = a->prec_vec();
        const Fluent_Vec&	pi = a->del_vec();


//        for ( unsigned j = 0; j < pi.size(); j++ )
//            if ( !m_reachable_atoms[pi[j]] )
//            {
//                applicable = false;
//                break;
//            }

        bool relevant = false;

        for ( unsigned k = 0; k < a->add_vec().size() && !relevant; k++ )
            if ( m_reachable_atoms[ a->add_vec()[k] ] )
                relevant = true;



		if ( ! relevant) continue;

		#ifdef DEBUG
		std::cout << "Applying " << a->signature() << std::endl;
        #endif
		
		// Apply effects
        //regression for delete the add_vec

        const Fluent_Vec& ai = a->prec_vec();
		for ( unsigned j = 0; j < ai.size(); j++ ) {

			if ( !m_reachable_atoms[ai[j]] ) {
				m_reach_next[ai[j]] = true;
				fixed_point = false;
			}
		}


		bool all_ce_applied = true;

		for( unsigned j = 0; j < a->ceff_vec().size(); j++ ) {

			const Fluent_Vec&	pi = a->ceff_vec()[j]->prec_vec();
			bool			applicable = true;

			for ( unsigned k = 0; k < pi.size(); k++ )
				if ( !m_reachable_atoms[pi[k]] ) {
					applicable = false;
					break;
				}
		
			if ( !applicable ) {
				all_ce_applied = false;
				continue;
			}

			const Fluent_Vec&	ai = a->ceff_vec()[j]->add_vec();

			for ( unsigned k = 0; k < ai.size(); k++ ) {

				if ( !m_reachable_atoms[ai[k]] ) {

					m_reach_next[ai[k]] = true;
					fixed_point = false;
				}
			}
	
		}
		if ( all_ce_applied ) m_action_mask.set(i);
	}
	m_reachable_atoms = m_reach_next;
	return fixed_point;
}

/** chao add
 *
 * @return
 */
    bool	Reachability_Test::apply_negation_actions() {

        bool fixed_point = true;
        m_reach_negation_next = m_reachable_negation_atoms;

        for ( unsigned i = 0; i < m_problem.negation_actions().size(); i++ )
        {
            if ( m_negation_action_mask.isset(i) ) continue;

            const Action* a = m_problem.negation_actions()[i];
            // Check if applicable
//		const Fluent_Vec&	pi = a->prec_vec();
            const Fluent_Vec&	pi = a->del_vec();


//        for ( unsigned j = 0; j < pi.size(); j++ )
//            if ( !m_reachable_atoms[pi[j]] )
//            {
//                applicable = false;
//                break;
//            }

            bool relevant = false;

            for ( unsigned k = 0; k < a->add_negation_vec().size() && !relevant; k++ )
                if ( m_reachable_negation_atoms[ a->add_negation_vec()[k] ] )
                    relevant = true;



            if ( ! relevant) continue;

#ifdef DEBUG
            std::cout << "Applying " << a->signature() << std::endl;
#endif

            // Apply effects
            //regression for delete the add_vec

            const Fluent_Vec& ai = a->prec_negation_vec();
            for ( unsigned j = 0; j < ai.size(); j++ ) {

                if ( !m_reachable_negation_atoms[ai[j]] ) {
                    m_reach_negation_next[ai[j]] = true;
                    fixed_point = false;
                }
            }


            bool all_ce_applied = true;

            for( unsigned j = 0; j < a->ceff_negation_vec().size(); j++ ) {

                const Fluent_Vec&	pi = a->ceff_negation_vec()[j]->prec_vec();
                bool			applicable = true;

                for ( unsigned k = 0; k < pi.size(); k++ )
                    if ( !m_reachable_negation_atoms[pi[k]] ) {
                        applicable = false;
                        break;
                    }

                if ( !applicable ) {
                    all_ce_applied = false;
                    continue;
                }

                const Fluent_Vec&	ai = a->ceff_negation_vec()[j]->add_vec();

                for ( unsigned k = 0; k < ai.size(); k++ ) {

                    if ( !m_reachable_negation_atoms[ai[k]] ) {

                        m_reach_negation_next[ai[k]] = true;
                        fixed_point = false;
                    }
                }

            }
            if ( all_ce_applied ) m_negation_action_mask.set(i);
        }
        m_reachable_negation_atoms = m_reach_negation_next;
        return fixed_point;
    }

void	Reachability_Test::print_reachable_atoms() {

	for (unsigned k = 0; k < m_reachable_atoms.size(); k++ )
		if ( m_reachable_atoms[k] )
			std::cout << m_problem.fluents()[k]->signature() << std::endl;	
}

void	Reachability_Test::get_reachable_actions( const Fluent_Vec& s, const Fluent_Vec& g,  Bit_Set& reach_actions ) {

	initialize(s);
 	
	while ( !apply_actions() )
	{
		#ifdef DEBUG
		std::cout << "Reachable atoms:" << std::endl;
		print_reachable_atoms();
		#endif

		if ( check( g ) )	
			break;
	}

	reach_actions.resize( m_problem.actions().size() );
	for ( unsigned i = 0; i < m_problem.actions().size(); i++ ){
		const Action* a = m_problem.actions()[i];		
		// Check if applicable
		const Fluent_Vec& pi = a->prec_vec();
		bool applicable = true;
		for ( unsigned j = 0; j < pi.size(); j++ )
			if ( !m_reachable_atoms[pi[j]] )
			{
				applicable = false;
				reach_actions.unset(i);
				break;
			}
		if(applicable) reach_actions.set(i);
		
	}
}

    void	Reachability_Test::get_reachable_actions_original( const Fluent_Vec& s, const Fluent_Vec& g,  Bit_Set& reach_actions ) {

        initialize(s);

        while ( !apply_actions_original() )
        {
#ifdef DEBUG
            std::cout << "Reachable atoms:" << std::endl;
		print_reachable_atoms();
#endif

            if ( check( g ) )
                break;
        }

        reach_actions.resize( m_problem.actions().size() );
        for ( unsigned i = 0; i < m_problem.actions().size(); i++ ){
            const Action* a = m_problem.actions()[i];
            // Check if applicable
            const Fluent_Vec& pi = a->prec_vec();
            bool applicable = true;
            for ( unsigned j = 0; j < pi.size(); j++ )
                if ( !m_reachable_atoms[pi[j]] )
                {
                    applicable = false;
                    reach_actions.unset(i);
                    break;
                }
            if(applicable) reach_actions.set(i);

        }
    }


    void	Reachability_Test::get_reachable_negation_actions( const Fluent_Vec& s, const Fluent_Vec& g,  Bit_Set& reach_actions ) {

        initialize_negation(s);

        while ( !apply_negation_actions() )
        {
#ifdef DEBUG
            std::cout << "Reachable atoms:" << std::endl;
		print_reachable_atoms();
#endif

            if ( check_negation( g ) )
                break;
        }

        reach_actions.resize( m_problem.negation_actions().size() );
        for ( unsigned i = 0; i < m_problem.negation_actions().size(); i++ ){
            const Action* a = m_problem.negation_actions()[i];
            // Check if applicable
            bool applicable = true;
            const Fluent_Vec& pi = a->del_negation_vec();

            for ( unsigned j = 0; j < pi.size(); j++ )
            if ( m_reachable_negation_atoms[pi[j]] )
            {
                applicable = false;
                break;
            }

            bool relevant = false;

            for ( unsigned k = 0; k < a->add_negation_vec().size() && !relevant; k++ )
                if ( m_reachable_negation_atoms[ a->add_negation_vec()[k] ] )
                    relevant = true;

            if(applicable&&relevant) reach_actions.set(i);

        }
    }

bool	Reachability_Test::is_reachable( const Fluent_Vec& s, const Fluent_Vec& g ) {

	initialize(s);

	#ifdef DEBUG
	std::cout << "Reachable atoms:" << std::endl;
	print_reachable_atoms();
	#endif	
	
	while ( !apply_actions() )
	{
		#ifdef DEBUG
		std::cout << "Reachable atoms:" << std::endl;
		print_reachable_atoms();
		#endif	

		if ( check( g ) )	
			return true;
	}
	return check(g);
}

bool	Reachability_Test::is_reachable( const Fluent_Vec& s, const Fluent_Vec& g, const unsigned op )
{
	initialize(s);

	#ifdef DEBUG
	std::cout << "Disabling operator " << m_problem.actions()[op]->signature() << std::endl;
	#endif
	m_action_mask.set(op);	
	#ifdef DEBUG
	std::cout << "Reachable atoms:" << std::endl;
	print_reachable_atoms();
	#endif	

	while ( !apply_actions() ) {
		#ifdef DEBUG
		std::cout << "Reachable atoms:" << std::endl;
		print_reachable_atoms();
		#endif	

		if ( check( g ) )	
			return true;
	}
	return check(g);
}

bool	Reachability_Test::is_reachable( const Fluent_Vec& s, const Fluent_Vec& g, const Bit_Set& excluded )
{
//    state_processed=state;
	initialize(s);

	m_action_mask.add(excluded);
	#ifdef DEBUG
	std::cout << "Reachable atoms:" << std::endl;
	print_reachable_atoms();
	#endif

	while ( !apply_actions() ) {
		#ifdef DEBUG
		std::cout << "Reachable atoms:" << std::endl;
		print_reachable_atoms();
		#endif

		if ( check( g ) )	
			return true;
	}
	return check(g);
}


bool	Reachability_Test::check( const Fluent_Vec& g )  {

	for ( unsigned i = 0; i < g.size(); i++ )
		if ( !m_reachable_atoms[ g[i] ] )
			return false;
	return true;
}

    bool	Reachability_Test::check_negation( const Fluent_Vec& g )  {
        bool goal_check= true;
        for (unsigned i=0;i<m_reachable_negation_atoms.size();i++){

            if (i%2==0&& m_reachable_negation_atoms[i] && std::find(g.begin(),g.end(),i/2)!=g.end()){
                continue;
            } else{
                if (i%2==0&&m_reachable_negation_atoms[i]){
                    goal_check= false;
                    if (m_reachable_negation_atoms[i+1]){
                        goal_check=true;
                    } else{
                        break;
                    }
                } else{
                    continue;
                }

            }

        }
        return  goal_check;
    }


}

}
