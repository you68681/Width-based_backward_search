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

#ifndef __NOVELTY_PARTITION__
#define __NOVELTY_PARTITION__

#include <aptk/search_prob.hxx>
#include <aptk/heuristic.hxx>
#include <aptk/ext_math.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <vector>
#include <deque>
#include <algorithm>   

namespace aptk {

namespace agnostic {


template <typename bwd_Search_Problem, typename Fwd_Search_Problem,typename Search_Node>
class Novelty_Partition : public Heuristic<State>{
public:

//
//	Novelty_Partition( const bwd_Search_Problem& prob, unsigned max_arity = 1, const unsigned max_MB = 2048 )
//		: Heuristic<State>( prob ), m_strips_model_bwd( prob.task() ), m_max_memory_size_MB_bwd(max_MB), m_always_full_state_bwd(false), m_partition_size(0), m_verbose( true ) {
//
//		set_arity(max_arity, 1);
//
//	}
//
//    Novelty_Partition( const Fwd_Search_Problem& prob, unsigned max_arity = 1, const unsigned max_MB = 2048 )
//            : Heuristic<State>( prob ), m_strips_model_fwd( prob.task() ), m_max_memory_size_MB_fwd(max_MB), m_always_full_state_fwd(false), m_partition_size(0), m_verbose( true ) {
//	    is_bwd = false;
//        set_arity(max_arity, 1);
//
//    }

    Novelty_Partition( const bwd_Search_Problem& prob_bwd,const Fwd_Search_Problem& prob_fwd, unsigned max_arity = 1, const unsigned max_MB = 2048 )
        : Heuristic<State>( prob_bwd,prob_fwd ),m_strips_model_bwd( prob_bwd.task() ), m_strips_model_fwd( prob_fwd.task() ),m_max_memory_size_MB_bwd(max_MB),m_max_memory_size_MB_fwd(max_MB),m_always_full_state_bwd(false),
        m_always_full_state_fwd(false),m_partition_size_bwd(0),m_partition_size_fwd(0), m_verbose_bwd( true ),m_verbose_fwd(true) {

        set_arity_bwd(max_arity, 1);

        set_arity_fwd(max_arity, 1);
        set_arity_novelty_fwd(max_arity);
        set_arity_novelty_bwd(max_arity);


    }
    virtual ~Novelty_Partition() {
	}


	void init_bwd() {
		typedef typename std::vector< std::vector<Search_Node*> >::iterator        Node_Vec_Ptr_It;
		typedef typename std::vector<Search_Node*>::iterator                       Node_Ptr_It;
				
		for(Node_Vec_Ptr_It it_p = m_nodes_tuples_by_partition_bwd.begin();it_p != m_nodes_tuples_by_partition_bwd.end(); it_p++)
			for(Node_Ptr_It it = it_p->begin(); it != it_p->end(); it++)		
				*it = NULL;
		init_novelty_bwd();

	}
    void init_fwd() {
        typedef typename std::vector< std::vector<Search_Node*> >::iterator        Node_Vec_Ptr_It;
        typedef typename std::vector<Search_Node*>::iterator                       Node_Ptr_It;

        for(Node_Vec_Ptr_It it_p = m_nodes_tuples_by_partition_fwd.begin();it_p != m_nodes_tuples_by_partition_fwd.end(); it_p++)
            for(Node_Ptr_It it = it_p->begin(); it != it_p->end(); it++)
                *it = NULL;
        init_novelty_fwd();
    }



	unsigned arity_bwd() const { return m_arity_bwd; }
	unsigned arity_fwd()  const{ return m_arity_fwd; }

	void set_full_state_computation_bwd( bool b )  { m_always_full_state_bwd = b ; }  //  m_always_full_state = b;
	void set_full_state_computation_fwd( bool b )  { m_always_full_state_fwd = b ; }

    void set_verbose_bwd( bool v ) { m_verbose_bwd = v; }
    void set_verbose_fwd( bool v ) { m_verbose_fwd = v; }


	unsigned& partition_size_bwd() {return m_partition_size_fwd;}
    unsigned& partition_size_fwd() {return m_partition_size_bwd;}


	bool is_partition_empty_bwd(unsigned partition) {return m_nodes_tuples_by_partition_bwd[partition].empty();}
    bool is_partition_empty_fwd(unsigned partition) {return m_nodes_tuples_by_partition_fwd[partition].empty();}
	
	Search_Node* table_bwd( unsigned partition, unsigned idx) { return m_nodes_tuples_by_partition_bwd[partition][idx]; }
    Search_Node* table_fwd( unsigned partition, unsigned idx) { return m_nodes_tuples_by_partition_fwd[partition][idx]; }


	void set_arity_bwd( unsigned max_arity, unsigned partition_size = 0 ){

	        m_partition_size_bwd = partition_size;
		m_arity_bwd = max_arity;
		m_num_tuples_bwd = 1;
		m_num_fluents_bwd = m_strips_model_bwd.num_fluents() ;           //m_strips_model.num_fluents()

		float size_novelty = ( (float) pow(m_num_fluents_bwd,m_arity_bwd) / 1024000.)  * (float) partition_size * sizeof(Search_Node*);
		//std::cout << "Try allocate size: "<< size_novelty<<" MB"<<std::endl;


		if(size_novelty >  m_max_memory_size_MB_bwd ){            //m_max_memory_size_MB
			m_arity_bwd = 1;
			size_novelty =  ( (float) pow(m_num_fluents_bwd,m_arity_bwd) / 1024000.) * (float) partition_size * sizeof(Search_Node*);

			std::cout<<"EXCEDED, m_arity downgraded to 1 --> size: "<< size_novelty<<" MB"<<std::endl;
		}

		for(unsigned k = 0; k < m_arity_bwd; k++)
			m_num_tuples_bwd *= m_num_fluents_bwd;

		m_nodes_tuples_by_partition_bwd.resize( partition_size+1 );

		for( unsigned i = 0; i < partition_size+1; i++ )
		    m_nodes_tuples_by_partition_bwd[i].clear();
	}




    void set_arity_fwd( unsigned max_arity, unsigned partition_size = 0 ){

        m_partition_size_fwd = partition_size;
        m_arity_fwd = max_arity;
        m_num_tuples_fwd = 1;
        m_num_fluents_fwd =   m_strips_model_fwd.num_fluents();           //m_strips_model.num_fluents()

        float size_novelty = ( (float) pow(m_num_fluents_fwd,m_arity_fwd) / 1024000.)  * (float) partition_size * sizeof(Search_Node*);
        //std::cout << "Try allocate size: "<< size_novelty<<" MB"<<std::endl;ab


        if(size_novelty >  m_max_memory_size_MB_fwd){            //m_max_memory_size_MB
            m_arity_fwd = 1;
            size_novelty =  ( (float) pow(m_num_fluents_fwd,m_arity_fwd) / 1024000.) * (float) partition_size * sizeof(Search_Node*);

            std::cout<<"EXCEDED, m_arity downgraded to 1 --> size: "<< size_novelty<<" MB"<<std::endl;
        }

        for(unsigned k = 0; k < m_arity_fwd; k++)
            m_num_tuples_fwd *= m_num_fluents_fwd;

        m_nodes_tuples_by_partition_fwd.resize( partition_size+1 );

        for( unsigned i = 0; i < partition_size+1; i++ )
            m_nodes_tuples_by_partition_fwd[i].clear();
    }



	virtual void eval_bwd( Search_Node* n, unsigned& h_val ) {
	
		compute_bwd( n, h_val );
	}
    virtual void eval_fwd( Search_Node* n, unsigned& h_val ) {

        compute_fwd( n, h_val );
    }
   /** chao edit
    *
    * @param n
    * @param h_val
    */
    void eval_novelty_fwd(  Search_Node* n, unsigned & h_val ) {
        compute_novelty_fwd( n, h_val );
    }
    void eval_novelty_bwd(  Search_Node* n, unsigned & h_val ) {
        compute_novelty_bwd( n, h_val );
    }


	virtual void eval_bwd( Search_Node* n, float& h_val ) {
		unsigned h;
		compute_bwd( n, h );
		h_val = h;
	}

    virtual void eval_fwd( Search_Node* n, float& h_val ) {
        unsigned h;
        compute_fwd( n, h );
        h_val = h;
    }


	virtual void eval( const State& s, unsigned& h_val ) {
	
		assert(true);
	}

	virtual void eval( const State& s, unsigned& h_val,  std::vector<Action_Idx>& pref_ops ) {
		assert(true);
	}



protected:
	void check_table_size_bwd( Search_Node* n ){

	    	if( m_partition_size_bwd < n->partition() ){
	    		m_nodes_tuples_by_partition_bwd.resize( n->partition() + 1 );
	    		m_partition_size_bwd = n->partition();
	    	}
		
		if(  m_nodes_tuples_by_partition_bwd[ n->partition() ].empty() )
			m_nodes_tuples_by_partition_bwd[ n->partition() ].resize( m_num_tuples_bwd, NULL );
	}



    void check_table_size_fwd( Search_Node* n ){

        if( m_partition_size_fwd < n->partition() ){
            m_nodes_tuples_by_partition_fwd.resize( n->partition() + 1 );
            m_partition_size_fwd = n->partition();
        }

        if(  m_nodes_tuples_by_partition_fwd[ n->partition() ].empty() )
            m_nodes_tuples_by_partition_fwd[ n->partition() ].resize( m_num_tuples_fwd, NULL );
    }



	/**
	 * If parent node is in the same space partition, check only new atoms,
	 * otherwise check all atoms in state
	 */
	void compute_bwd(  Search_Node* n, unsigned& novelty )
	{

		novelty = (float) m_arity_bwd+1;

		if( n->partition() == std::numeric_limits<unsigned>::max() ) return;
		
		check_table_size_bwd( n );

		for(unsigned i = 1; i <= m_arity_bwd; i++){
#ifdef DEBUG
			if ( m_verbose )
				std::cout << "search state node: "<<&(n)<<std::endl;
#endif 	
			
			bool new_covers;


			if(n->parent() == nullptr || m_always_full_state_bwd )
				new_covers = cover_tuples_bwd( n, i );
			else
				new_covers = (n->partition() == n->parent()->partition()) ?  cover_tuples_op_bwd( n, i ) : cover_tuples_bwd( n, i );


			
#ifdef DEBUG
			if(m_verbose && !new_covers)	
				std::cout << "\t \t PRUNE! search node: "<<&(n)<<std::endl;
#endif 	
			if ( new_covers )
				if(i < novelty )
					novelty = i;
		}
	}

    void compute_fwd(  Search_Node* n, unsigned& novelty )
    {

        novelty = (float) m_arity_fwd+1;

        if( n->partition() == std::numeric_limits<unsigned>::max() ) return;

        check_table_size_fwd( n );

        for(unsigned i = 1; i <= m_arity_fwd; i++){
#ifdef DEBUG
            if ( m_verbose )
				std::cout << "search state node: "<<&(n)<<std::endl;
#endif

            bool new_covers;


            if(n->parent() == nullptr ||  m_always_full_state_fwd)
                new_covers = cover_tuples_fwd( n, i );
            else
                new_covers = (n->partition() == n->parent()->partition()) ?  cover_tuples_op_fwd( n, i ) : cover_tuples_fwd( n, i );



#ifdef DEBUG
            if(m_verbose && !new_covers)
				std::cout << "\t \t PRUNE! search node: "<<&(n)<<std::endl;
#endif
            if ( new_covers )
                if(i < novelty )
                    novelty = i;
        }
    }






	       
	bool cover_tuples_bwd( Search_Node* n, unsigned arity  )
	{
		const bool has_state = n->has_state();
		
		if(!has_state)
		    n->parent()->state()->progress_lazy_state_bwd(  m_strips_model_bwd.actions()[ n->action() ] ) ;
       // n->parent()->state()->progress_lazy_state(  m_strips_model.actions()[ n->action() ] );
				

		Fluent_Vec& fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();      

		bool new_covers = false;

		std::vector<unsigned> tuple( arity );

 		unsigned n_combinations = aptk::unrolled_pow(  fl.size() , arity );

		for( unsigned idx = 0; idx < n_combinations; idx++ ){
			/**
			 * get tuples from indexes
			 */
			idx2tuple( tuple, fl, idx, arity );

			/**
			 * Check if tuple is covered
			 */
			
			unsigned tuple_idx;

			
			if (arity==1) {
				tuple_idx = tuple2idx_bwd( tuple, arity );
				
			} else 	if(arity == 2 ){			
				if( tuple[0] == tuple[1] ) continue; // don't check singleton tuples
				tuple_idx = tuple2idx_size2_bwd( tuple, arity );
			} else {
				
				// If all elements in the tuple are equal, ignore the tuple
				if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x){ return x != tuple[0]; }  )) continue;
				tuple_idx = tuple2idx_bwd( tuple, arity );
			}
			
			

			/**
			 * new_tuple if
			 * -> none was registered
			 * OR
			 * -> n better than old_n
			 */

			auto& n_seen = m_nodes_tuples_by_partition_bwd[ n->partition() ][ tuple_idx ];

			if (!n_seen || is_better(n_seen,n)) {
				
				n_seen = (Search_Node*) n;
				new_covers = true;
				
				
#ifdef DEBUG
				if ( m_verbose ) {
					std::cout<<"\t NEW!! : ";
					for(unsigned i = 0; i < arity; i++){
						std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
					}
					std::cout << std::endl;
				}
#endif
			}

		}

		if(!has_state)
		   n->parent()->state()->regress_lazy_state_bwd( m_strips_model_bwd.actions()[ n->action() ] ) ;
       // n->parent()->state()->regress_lazy_state( m_strips_model.actions()[ n->action() ] );
		
		return new_covers;

	}


    bool cover_tuples_fwd( Search_Node* n, unsigned arity  )
    {
        const bool has_state = n->has_state();

        if(!has_state)
            n->parent()->state()->progress_lazy_state_fwd(  m_strips_model_fwd.actions()[ n->action() ] );
        // n->parent()->state()->progress_lazy_state(  m_strips_model.actions()[ n->action() ] );


        Fluent_Vec& fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

        bool new_covers = false;

        std::vector<unsigned> tuple( arity );

        unsigned n_combinations = aptk::unrolled_pow(  fl.size() , arity );

        for( unsigned idx = 0; idx < n_combinations; idx++ ){
            /**
             * get tuples from indexes
             */
            idx2tuple( tuple, fl, idx, arity );

            /**
             * Check if tuple is covered
             */

            unsigned tuple_idx;


            if (arity==1) {
                tuple_idx = tuple2idx_fwd( tuple, arity );

            } else 	if(arity == 2 ){
                if( tuple[0] == tuple[1] ) continue; // don't check singleton tuples
                tuple_idx = tuple2idx_size2_fwd( tuple, arity );
            } else {

                // If all elements in the tuple are equal, ignore the tuple
                if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x){ return x != tuple[0]; }  )) continue;
                tuple_idx = tuple2idx_fwd( tuple, arity );
            }



            /**
             * new_tuple if
             * -> none was registered
             * OR
             * -> n better than old_n
             */

            auto& n_seen = m_nodes_tuples_by_partition_fwd[ n->partition() ][ tuple_idx ];

            if (!n_seen || is_better(n_seen,n)) {

                n_seen = (Search_Node*) n;
                new_covers = true;


#ifdef DEBUG
                if ( m_verbose ) {
					std::cout<<"\t NEW!! : ";
					for(unsigned i = 0; i < arity; i++){
						std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
					}
					std::cout << std::endl;
				}
#endif
            }

        }

        if(!has_state)
           n->parent()->state()->regress_lazy_state_fwd( m_strips_model_fwd.actions()[ n->action() ] );
        // n->parent()->state()->regress_lazy_state( m_strips_model.actions()[ n->action() ] );

        return new_covers;

    }

	/**
	 * Instead of checking the whole state, checks the new atoms permutations only!
	 */
	
	bool    cover_tuples_op_bwd( Search_Node* n, unsigned arity  ) {

        const bool has_state = n->has_state();

        static Fluent_Vec new_atom_vec;
        const Action *a = m_strips_model_bwd.actions()[n->action()];
        if (a->has_ceff()) {
            static Fluent_Set new_atom_set(m_strips_model_bwd.num_fluents() + 1 );      //m_strips_model.num_fluents()+1
            new_atom_set.reset();
            new_atom_vec.clear();
            for (Fluent_Vec::const_iterator it = a->add_vec().begin(); it != a->add_vec().end(); it++) {
                if (new_atom_set.isset(*it)) continue;

                new_atom_vec.push_back(*it);
                new_atom_set.set(*it);

            }
            for (unsigned i = 0; i < a->ceff_vec().size(); i++) {
                Conditional_Effect *ce = a->ceff_vec()[i];
                if (ce->can_be_applied_on(*(n->parent()->state())))
                    for (Fluent_Vec::iterator it = ce->add_vec().begin(); it != ce->add_vec().end(); it++) {
                        {
                            if (new_atom_set.isset(*it)) continue;

                            new_atom_vec.push_back(*it);
                            new_atom_set.set(*it);
                        }
                    }

            }
        }

        //const Fluent_Vec& add = a->has_ceff() ? new_atom_vec : a->add_vec();
        const Fluent_Vec &add = a->has_ceff() ? new_atom_vec : a->prec_vec();

        if (!has_state)
          n->parent()->state()->progress_lazy_state_bwd(m_strips_model_bwd.actions()[n->action()]);

        //n->parent()->state()->progress_lazy_state(  m_strips_model.actions()[ n->action() ] );

        Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

        bool new_covers = false;

        assert (arity > 0);

        std::vector<unsigned> tuple(arity);

        unsigned atoms_arity = arity - 1;
        unsigned n_combinations = aptk::unrolled_pow(fl.size(), atoms_arity);


        for (Fluent_Vec::const_iterator it_add = add.begin();
             it_add != add.end(); it_add++) {


            for (unsigned idx = 0; idx < n_combinations; idx++) {

                tuple[atoms_arity] = *it_add;
                /**
                 * Check if tuple is covered
                 */
                unsigned tuple_idx;


                if (arity == 1) {
                    tuple_idx = tuple2idx_bwd(tuple, arity);

                } else if (arity == 2) {
                    tuple[0] = fl[idx];
                    if (tuple[0] == tuple[1]) continue; // don't check singleton tuples
                    tuple_idx = tuple2idx_size2_bwd(tuple, arity);
                } else {

                    // If all elements in the tuple are equal, ignore the tuple
                    if (std::any_of(tuple.cbegin(), tuple.cend(),
                                    [&tuple](unsigned x) { return x != tuple[0]; }))
                        continue;
                    /**
                     * get tuples from indexes
                     */
                    idx2tuple(tuple, fl, idx, atoms_arity);


                    tuple_idx = tuple2idx_bwd(tuple, arity);
                }



                /**
                 * new_tuple if
                 * -> none was registered
                 * OR
                 * -> n better than old_n
                 */

                auto &n_seen = m_nodes_tuples_by_partition_bwd[n->partition()][tuple_idx];

                if (!n_seen || is_better(n_seen, n)) {

                    n_seen = (Search_Node *) n;
                    new_covers = true;


#ifdef DEBUG
                    if ( m_verbose ) {
                        std::cout<<"\t NEW!! : ";
                        for(unsigned i = 0; i < arity; i++){
                            std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
                        }
                        std::cout << std::endl;
                    }
#endif
                }

            }
        }

        if (!has_state)
             n->parent()->state()->regress_lazy_state_bwd(m_strips_model_bwd.actions()[n->action()]);


        return new_covers;

	}

    bool    cover_tuples_op_fwd( Search_Node* n, unsigned arity  ) {

        const bool has_state = n->has_state();

        static Fluent_Vec new_atom_vec;
        const Action *a =m_strips_model_fwd.actions()[n->action()];
        if (a->has_ceff()) {
            static Fluent_Set new_atom_set(m_strips_model_fwd.num_fluents() +1 );      //m_strips_model.num_fluents()+1
            new_atom_set.reset();
            new_atom_vec.clear();
            for (Fluent_Vec::const_iterator it = a->add_vec().begin(); it != a->add_vec().end(); it++) {
                if (new_atom_set.isset(*it)) continue;

                new_atom_vec.push_back(*it);
                new_atom_set.set(*it);

            }
            for (unsigned i = 0; i < a->ceff_vec().size(); i++) {
                Conditional_Effect *ce = a->ceff_vec()[i];
                if (ce->can_be_applied_on(*(n->parent()->state())))
                    for (Fluent_Vec::iterator it = ce->add_vec().begin(); it != ce->add_vec().end(); it++) {
                        {
                            if (new_atom_set.isset(*it)) continue;

                            new_atom_vec.push_back(*it);
                            new_atom_set.set(*it);
                        }
                    }

            }
        }
        /** forward search
         *
         */
        const Fluent_Vec& add = a->has_ceff() ? new_atom_vec : a->add_vec();
        //const Fluent_Vec &add = a->has_ceff() ? new_atom_vec : a->prec_vec();

        if (!has_state)
            n->parent()->state()->progress_lazy_state_fwd(m_strips_model_fwd.actions()[n->action()]);
        //n->parent()->state()->progress_lazy_state(  m_strips_model.actions()[ n->action() ] );

        Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

        bool new_covers = false;

        assert (arity > 0);

        std::vector<unsigned> tuple(arity);

        unsigned atoms_arity = arity - 1;
        unsigned n_combinations = aptk::unrolled_pow(fl.size(), atoms_arity);


        for (Fluent_Vec::const_iterator it_add = add.begin();
             it_add != add.end(); it_add++) {


            for (unsigned idx = 0; idx < n_combinations; idx++) {

                tuple[atoms_arity] = *it_add;
                /**
                 * Check if tuple is covered
                 */
                unsigned tuple_idx;


                if (arity == 1) {
                    tuple_idx = tuple2idx_fwd(tuple, arity);

                } else if (arity == 2) {
                    tuple[0] = fl[idx];
                    if (tuple[0] == tuple[1]) continue; // don't check singleton tuples
                    tuple_idx = tuple2idx_size2_fwd(tuple, arity);
                } else {

                    // If all elements in the tuple are equal, ignore the tuple
                    if (std::any_of(tuple.cbegin(), tuple.cend(),
                                    [&tuple](unsigned x) { return x != tuple[0]; }))
                        continue;
                    /**
                     * get tuples from indexes
                     */
                    idx2tuple(tuple, fl, idx, atoms_arity);


                    tuple_idx = tuple2idx_fwd(tuple, arity);
                }



                /**
                 * new_tuple if
                 * -> none was registered
                 * OR
                 * -> n better than old_n
                 */

                auto &n_seen = m_nodes_tuples_by_partition_fwd[n->partition()][tuple_idx];

                if (!n_seen || is_better(n_seen, n)) {

                    n_seen = (Search_Node *) n;
                    new_covers = true;


#ifdef DEBUG
                    if ( m_verbose ) {
                        std::cout<<"\t NEW!! : ";
                        for(unsigned i = 0; i < arity; i++){
                            std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
                        }
                        std::cout << std::endl;
                    }
#endif
                }

            }
        }

        if (!has_state)
           n->parent()->state()->regress_lazy_state_fwd(m_strips_model_fwd.actions()[n->action()]);


        return new_covers;

    }



	inline unsigned  tuple2idx_size2_bwd( std::vector<unsigned>& indexes, unsigned arity) const
	{
		unsigned min = indexes[0] <= indexes[1] ? indexes[0] : indexes[1];
		unsigned max = indexes[0] <= indexes[1] ? indexes[1] : indexes[0];
		return min + max*m_num_fluents_bwd;
			
	}


    inline unsigned  tuple2idx_size2_fwd( std::vector<unsigned>& indexes, unsigned arity) const
    {
        unsigned min = indexes[0] <= indexes[1] ? indexes[0] : indexes[1];
        unsigned max = indexes[0] <= indexes[1] ? indexes[1] : indexes[0];
        return min + max*m_num_fluents_fwd;

    }

	inline unsigned  tuple2idx_bwd( std::vector<unsigned>& indexes, unsigned arity) const
	{
		unsigned idx=0;
		unsigned dimension = 1;

		std::sort(indexes.begin(), indexes.end());
		for(int i = arity-1; i >= 0; i--)
			{
				idx += indexes[ i ] * dimension;
				dimension*= m_num_fluents_bwd;
			}

		return idx;

	}
    inline unsigned  tuple2idx_fwd( std::vector<unsigned>& indexes, unsigned arity) const
    {
        unsigned idx=0;
        unsigned dimension = 1;

        std::sort(indexes.begin(), indexes.end());
        for(int i = arity-1; i >= 0; i--)
        {
            idx += indexes[ i ] * dimension;
            dimension*= m_num_fluents_fwd;
        }

        return idx;

    }

	inline void      idx2tuple( std::vector<unsigned>& tuple, Fluent_Vec& fl, unsigned idx, unsigned arity ) const
	{
		unsigned next_idx, div;
		unsigned current_idx = idx;
		int n_atoms =  fl.size();

		for(int i = arity-1; i >= 0 ; i--) {
			// MRJ: Let's use the fast version
			// n_atoms = 10
			// i = 3 (w=4) -> div = 1000
			// i = 2 (w=3) -> div = 100
			// i = 1 (w=2) -> div = 10
			// i = 0 (w=1) -> div = 1
			div = aptk::unrolled_pow( n_atoms , i );

			if ( current_idx < div ) {
				next_idx = current_idx;
				current_idx = 0;
			}
			else {
				// current index : 32
				// i = 3 -> next_idx = 32 % 1000 = 32, 32 / 1000 = 0
				// i = 2 -> next_idx = 32 % 100 = 32, 32 / 100 = 0
				// i = 1 -> next_idx = 32 % 10 = 2, 32 / 10 = 3
				// i = 0 -> next_idx = 32 % 1 = 0, 32 / 1 = 32
				next_idx = current_idx % div;
				const int div_res = current_idx / div;
				// if current_idx is zero and is the last index, then take next_idx
				// i = 3, current_idx = ( 32 / 1000 != 0 || i != 0 ) ? 32 / 1000 : 32 = ( F || T ) ? 0 : 32 = 0
				current_idx = ( div_res != 0 || i != 0) ? div_res : next_idx;
			}
			tuple[ i ] = fl[ current_idx ];

			current_idx = next_idx;
		}
	}

	inline bool      is_better( Search_Node* n,const Search_Node* new_n ) const {
		//return false;
		return new_n->is_better( n );		
	}



    /** chao edit code
     *
     */


    void init_novelty_fwd() {
        typedef typename std::vector<Search_Node*>::iterator                       Node_Ptr_It;

        for(Node_Ptr_It it = m_nodes_tuples_novelty_fwd.begin(); it != m_nodes_tuples_novelty_fwd.end(); it++)
            *it = NULL;

    }
    void init_novelty_bwd() {
        typedef typename std::vector<Search_Node*>::iterator                       Node_Ptr_It;

        for(Node_Ptr_It it = m_nodes_tuples_novelty_bwd.begin(); it != m_nodes_tuples_novelty_bwd.end(); it++)
            *it = NULL;

    }


    unsigned arity_novelty() const { return m_arity_novelty; }

    void set_arity_novelty_fwd( unsigned max_arity ){

        m_arity_novelty = max_arity;
        m_num_tuples_novelty= 1;
        m_num_fluents_novelty= m_strips_model_fwd.num_fluents();

        float size_novelty = ( (float) pow(m_num_fluents_novelty,m_arity_novelty) / 1024000.) * sizeof(Search_Node*);
        if ( m_verbose_fwd )
            std::cout << "Try allocate size: "<< size_novelty<<" MB"<<std::endl;
        if(size_novelty > m_max_memory_size_MB_fwd){
            m_arity_novelty = 1;
            size_novelty =  ( (float) pow(m_num_fluents_novelty,m_arity_novelty) / 1024000.) * sizeof(Search_Node*);
            if ( m_verbose_fwd )
                std::cout<<"EXCEDED, m_arity downgraded to 1 --> size: "<< size_novelty<<" MB"<<std::endl;
        }

        for(unsigned k = 0; k < m_arity_novelty; k++)
            m_num_tuples_novelty *= m_num_fluents_novelty;

        m_nodes_tuples_novelty_fwd.resize(m_num_tuples_novelty, NULL);

    }
    void set_arity_novelty_bwd( unsigned max_arity ){

        m_arity_novelty = max_arity;
        m_num_tuples_novelty = 1;
        m_num_fluents_novelty = m_strips_model_bwd.num_fluents();

        float size_novelty = ( (float) pow(m_num_fluents_novelty,m_arity_novelty) / 1024000.) * sizeof(Search_Node*);
        if ( m_verbose_novelty )
            std::cout << "Try allocate size: "<< size_novelty<<" MB"<<std::endl;
        if(size_novelty > m_max_memory_size_MB_bwd){
            m_arity_novelty = 1;
            size_novelty =  ( (float) pow(m_num_fluents_novelty,m_arity_novelty) / 1024000.) * sizeof(Search_Node*);
            if ( m_verbose_novelty )
                std::cout<<"EXCEDED, m_arity downgraded to 1 --> size: "<< size_novelty<<" MB"<<std::endl;
        }

        for(unsigned k = 0; k < m_arity_novelty; k++)
            m_num_tuples_novelty *= m_num_fluents_novelty;

        m_nodes_tuples_novelty_bwd.resize(m_num_tuples_novelty, NULL);

    }




    void compute_novelty_fwd(  Search_Node* n, unsigned & novelty )
    {

        novelty = (float) m_arity_novelty+1;
        for(unsigned i = 1; i <= m_arity_novelty; i++){

#ifdef DEBUG
            if ( m_verbose )
				std::cout << "search node: "<< n <<std::endl;
#endif
            bool new_covers = n->action() == no_op ? cover_tuples_novelty_fwd( n, i ) : cover_tuples_op_novelty_fwd( n, i );

#ifdef DEBUG
            if(m_verbose && !new_covers)
				std::cout << "\t \t PRUNE! search node: "<< n <<std::endl;
#endif
            if ( new_covers )
                if(i < novelty )
                    novelty = i;
        }
    }
    void compute_novelty_bwd(  Search_Node* n, unsigned & novelty )
    {

        novelty = (float) m_arity_novelty+1;
        for(unsigned i = 1; i <= m_arity_novelty; i++){

#ifdef DEBUG
            if ( m_verbose )
				std::cout << "search node: "<< n <<std::endl;
#endif
            bool new_covers = n->action() == no_op ? cover_tuples_novelty_bwd( n, i ) : cover_tuples_op_novelty_bwd( n, i );

#ifdef DEBUG
            if(m_verbose && !new_covers)
				std::cout << "\t \t PRUNE! search node: "<< n <<std::endl;
#endif
            if ( new_covers )
                if(i < novelty )
                    novelty = i;
        }
    }

    bool    cover_tuples_op_novelty_fwd( Search_Node* n, unsigned arity )
    {


        const bool has_state = n->has_state();

        static Fluent_Vec new_atom_vec;
        const Action* a = m_strips_model_fwd.actions()[ n->action() ];
        if( a->has_ceff() )
        {
            static Fluent_Set new_atom_set( m_strips_model_fwd.num_fluents()+1 );
            new_atom_set.reset();
            new_atom_vec.clear();
		    for(Fluent_Vec::const_iterator it = a->add_vec().begin(); it != a->add_vec().end(); it++)
//            for(Fluent_Vec::const_iterator it = a->prec_vec().begin(); it != a->prec_vec().end(); it++)
            {
                if ( new_atom_set.isset( *it ) ) continue;

                new_atom_vec.push_back( *it );
                new_atom_set.set( *it );

            }
            for( unsigned i = 0; i < a->ceff_vec().size(); i++ )
            {
                Conditional_Effect* ce = a->ceff_vec()[i];
                if( ce->can_be_applied_on( *(n->parent()->state()) ) )
//            if( ce->can_be_applied_on( *(n->parent()->state()) ) )
//			  for(Fluent_Vec::iterator it = ce->add_vec().begin(); it != ce->add_vec().end(); it++){
                    for(Fluent_Vec::iterator it = ce->prec_vec().begin(); it != ce->prec_vec().end(); it++){
                        {
                            if ( new_atom_set.isset( *it ) ) continue;

                            new_atom_vec.push_back( *it );
                            new_atom_set.set( *it );
                        }
                    }

            }
        }
		const Fluent_Vec& add = a->has_ceff() ? new_atom_vec : a->add_vec();
//        const Fluent_Vec& add = a->has_ceff() ? new_atom_vec : a->prec_vec();

        if(!has_state)
            n->parent()->state()->progress_lazy_state_fwd(  m_strips_model_fwd.actions()[ n->action() ] );

        Fluent_Vec& fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

        bool new_covers = false;

        assert ( arity > 0 );

        std::vector<unsigned> tuple( arity );

        unsigned atoms_arity = arity - 1;
        unsigned n_combinations = aptk::unrolled_pow(  fl.size() , atoms_arity );


        for ( Fluent_Vec::const_iterator it_add = add.begin();
              it_add != add.end(); it_add++ )
        {



            for( unsigned idx = 0; idx < n_combinations; idx++ ){

                tuple[ atoms_arity ] = *it_add;

                /**
                 * get tuples from indexes
                 */
                if(atoms_arity > 0)
                    idx2tuple_novelty_fwd( tuple, fl, idx, atoms_arity );

                /**
                 * Check if tuple is covered
                 */
                unsigned tuple_idx;


                if (arity==1) {
                    tuple_idx = tuple2idx_novelty_fwd( tuple, arity );

                } else 	if(arity == 2 ){
                    tuple[0] = fl[idx];
                    if( tuple[0] == tuple[1] ) continue; // don't check singleton tuples
                    tuple_idx = tuple2idx_size2_novelty_fwd( tuple, arity );
                } else {

                    // If all elements in the tuple are equal, ignore the tuple
                    if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x){ return x != tuple[0]; }  )) continue;
                    /**
                     * get tuples from indexes
                     */
                    idx2tuple_novelty_fwd( tuple, fl, idx, atoms_arity );


                    tuple_idx = tuple2idx_novelty_fwd( tuple, arity );
                }



                /**
                 * new_tuple if
                 * -> none was registered
                 * OR
                 * -> n better than old_n
                 */
                auto& n_seen = m_nodes_tuples_novelty_fwd[ tuple_idx ];

                if (!n_seen || is_better(n_seen,n)) {

                    n_seen = (Search_Node*) n;
                    new_covers = true;


#ifdef DEBUG
                    if ( m_verbose ) {
							std::cout<<"\t NEW!! : ";
							for(unsigned i = 0; i < arity; i++){
								std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
							}
							std::cout << " by state: "<< m_nodes_tuples[ tuple_idx ] << "" ;
							std::cout << std::endl;
						}
#endif
                }
                else
                {
#ifdef DEBUG
                    if ( m_verbose ) {
								std::cout<<"\t TUPLE COVERED: ";
								for(unsigned i = 0; i < arity; i++){
									std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
								}

								std::cout << " by state: "<< m_nodes_tuples[ tuple_idx ] << "" <<std::flush;

								std::cout<< std::endl;
							}
#endif
                }

            }
        }

        if(!has_state)
            n->parent()->state()->regress_lazy_state_fwd( m_strips_model_fwd.actions()[ n->action() ] );

        return new_covers;
    }

    bool    cover_tuples_op_novelty_bwd( Search_Node* n, unsigned arity )
    {


        const bool has_state = n->has_state();

        static Fluent_Vec new_atom_vec;
        const Action* a = m_strips_model_bwd.actions()[ n->action() ];
        if( a->has_ceff() )
        {
            static Fluent_Set new_atom_set( m_strips_model_bwd.num_fluents()+1 );
            new_atom_set.reset();
            new_atom_vec.clear();
//		    for(Fluent_Vec::const_iterator it = a->add_vec().begin(); it != a->add_vec().end(); it++)
            for(Fluent_Vec::const_iterator it = a->prec_vec().begin(); it != a->prec_vec().end(); it++)
            {
                if ( new_atom_set.isset( *it ) ) continue;

                new_atom_vec.push_back( *it );
                new_atom_set.set( *it );

            }
            for( unsigned i = 0; i < a->ceff_vec().size(); i++ )
            {
                Conditional_Effect* ce = a->ceff_vec()[i];
                if( ce->can_be_applied_on( *(n->parent()->state()) ) )
//            if( ce->can_be_applied_on( *(n->parent()->state()) ) )
//			  for(Fluent_Vec::iterator it = ce->add_vec().begin(); it != ce->add_vec().end(); it++){
                    for(Fluent_Vec::iterator it = ce->prec_vec().begin(); it != ce->prec_vec().end(); it++){
                        {
                            if ( new_atom_set.isset( *it ) ) continue;

                            new_atom_vec.push_back( *it );
                            new_atom_set.set( *it );
                        }
                    }

            }
        }
//		const Fluent_Vec& add = a->has_ceff() ? new_atom_vec : a->add_vec();
        const Fluent_Vec& add = a->has_ceff() ? new_atom_vec : a->prec_vec();

        if(!has_state)
            n->parent()->state()->progress_lazy_state_bwd(  m_strips_model_bwd.actions()[ n->action() ] );

        Fluent_Vec& fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

        bool new_covers = false;

        assert ( arity > 0 );

        std::vector<unsigned> tuple( arity );

        unsigned atoms_arity = arity - 1;
        unsigned n_combinations = aptk::unrolled_pow(  fl.size() , atoms_arity );


        for ( Fluent_Vec::const_iterator it_add = add.begin();
              it_add != add.end(); it_add++ )
        {



            for( unsigned idx = 0; idx < n_combinations; idx++ ){

                tuple[ atoms_arity ] = *it_add;

                /**
                 * get tuples from indexes
                 */
                if(atoms_arity > 0)
                    idx2tuple_novelty_bwd( tuple, fl, idx, atoms_arity );

                /**
                 * Check if tuple is covered
                 */
                unsigned tuple_idx;


                if (arity==1) {
                    tuple_idx = tuple2idx_novelty_bwd( tuple, arity );

                } else 	if(arity == 2 ){
                    tuple[0] = fl[idx];
                    if( tuple[0] == tuple[1] ) continue; // don't check singleton tuples
                    tuple_idx = tuple2idx_size2_novelty_bwd( tuple, arity );
                } else {

                    // If all elements in the tuple are equal, ignore the tuple
                    if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x){ return x != tuple[0]; }  )) continue;
                    /**
                     * get tuples from indexes
                     */
                    idx2tuple_novelty_bwd( tuple, fl, idx, atoms_arity );


                    tuple_idx = tuple2idx_novelty_bwd( tuple, arity );
                }



                /**
                 * new_tuple if
                 * -> none was registered
                 * OR
                 * -> n better than old_n
                 */
                auto& n_seen = m_nodes_tuples_novelty_bwd[ tuple_idx ];

                if (!n_seen || is_better(n_seen,n)) {

                    n_seen = (Search_Node*) n;
                    new_covers = true;


#ifdef DEBUG
                    if ( m_verbose ) {
							std::cout<<"\t NEW!! : ";
							for(unsigned i = 0; i < arity; i++){
								std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
							}
							std::cout << " by state: "<< m_nodes_tuples[ tuple_idx ] << "" ;
							std::cout << std::endl;
						}
#endif
                }
                else
                {
#ifdef DEBUG
                    if ( m_verbose ) {
								std::cout<<"\t TUPLE COVERED: ";
								for(unsigned i = 0; i < arity; i++){
									std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
								}

								std::cout << " by state: "<< m_nodes_tuples[ tuple_idx ] << "" <<std::flush;

								std::cout<< std::endl;
							}
#endif
                }

            }
        }

        if(!has_state)
            n->parent()->state()->regress_lazy_state_bwd( m_strips_model_bwd.actions()[ n->action() ] );

        return new_covers;
    }


    bool cover_tuples_novelty_fwd( Search_Node* n, unsigned arity  )
    {
        const bool has_state = n->has_state();

        if(!has_state)
            n->parent()->state()->progress_lazy_state_fwd(  m_strips_model_fwd.actions()[ n->action() ] );

        Fluent_Vec& fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

        bool new_covers = false;

        std::vector<unsigned> tuple( arity );

        unsigned n_combinations = aptk::unrolled_pow(  fl.size() , arity );


#ifdef DEBUG
        if ( m_verbose )
			std::cout<< n << " covers: " << std::endl;
#endif

        for( unsigned idx = 0; idx < n_combinations; idx++ ){
            /**
             * get tuples from indexes
             */
            idx2tuple_novelty_fwd( tuple, fl, idx, arity );

            /**
             * Check if tuple is covered
             */
            unsigned tuple_idx;


            if (arity==1) {
                tuple_idx = tuple2idx_novelty_fwd( tuple, arity );

            } else 	if(arity == 2 ){
                if( tuple[0] == tuple[1] ) continue; // don't check singleton tuples
                tuple_idx = tuple2idx_size2_novelty_fwd( tuple, arity );
            } else {

                // If all elements in the tuple are equal, ignore the tuple
                if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x){ return x != tuple[0]; }  )) continue;
                tuple_idx = tuple2idx_novelty_fwd( tuple, arity );
            }

            /**
             * new_tuple if
             * -> none was registered
             * OR
             * -> n better than old_n
             */

            auto& n_seen = m_nodes_tuples_novelty_fwd[ tuple_idx ];

            if (!n_seen || is_better(n_seen,n)) {

                n_seen = (Search_Node*) n;

                new_covers = true;
#ifdef DEBUG
                if ( m_verbose ) {
					std::cout<<"\t";
					for(unsigned i = 0; i < arity; i++){
						std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
					}
					std::cout << std::endl;
				}
#endif
            }

        }
        if(!has_state)
            n->parent()->state()->regress_lazy_state_fwd( m_strips_model_fwd.actions()[ n->action() ] );

        return new_covers;

    }

    bool cover_tuples_novelty_bwd( Search_Node* n, unsigned arity  )
    {
        const bool has_state = n->has_state();

        if(!has_state)
            n->parent()->state()->progress_lazy_state_bwd(  m_strips_model_bwd.actions()[ n->action() ] );

        Fluent_Vec& fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

        bool new_covers = false;

        std::vector<unsigned> tuple( arity );

        unsigned n_combinations = aptk::unrolled_pow(  fl.size() , arity );


#ifdef DEBUG
        if ( m_verbose )
			std::cout<< n << " covers: " << std::endl;
#endif

        for( unsigned idx = 0; idx < n_combinations; idx++ ){
            /**
             * get tuples from indexes
             */
            idx2tuple_novelty_bwd( tuple, fl, idx, arity );

            /**
             * Check if tuple is covered
             */
            unsigned tuple_idx;


            if (arity==1) {
                tuple_idx = tuple2idx_novelty_bwd( tuple, arity );

            } else 	if(arity == 2 ){
                if( tuple[0] == tuple[1] ) continue; // don't check singleton tuples
                tuple_idx = tuple2idx_size2_novelty_bwd( tuple, arity );
            } else {

                // If all elements in the tuple are equal, ignore the tuple
                if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x){ return x != tuple[0]; }  )) continue;
                tuple_idx = tuple2idx_novelty_bwd( tuple, arity );
            }

            /**
             * new_tuple if
             * -> none was registered
             * OR
             * -> n better than old_n
             */

            auto& n_seen = m_nodes_tuples_novelty_bwd[ tuple_idx ];

            if (!n_seen || is_better(n_seen,n)) {

                n_seen = (Search_Node*) n;

                new_covers = true;
#ifdef DEBUG
                if ( m_verbose ) {
					std::cout<<"\t";
					for(unsigned i = 0; i < arity; i++){
						std::cout<< m_strips_model.fluents()[ tuple[i] ]->signature()<<"  ";
					}
					std::cout << std::endl;
				}
#endif
            }

        }
        if(!has_state)
            n->parent()->state()->regress_lazy_state_bwd( m_strips_model_bwd.actions()[ n->action() ] );

        return new_covers;

    }

    //specialized version for tuples of size 2
    inline unsigned  tuple2idx_size2_novelty_fwd( std::vector<unsigned>& indexes, unsigned arity) const
    {
        unsigned min = indexes[0] <= indexes[1] ? indexes[0] : indexes[1];
        unsigned max = indexes[0] <= indexes[1] ? indexes[1] : indexes[0];
        return min + max*m_num_fluents_novelty;

    }
    inline unsigned  tuple2idx_size2_novelty_bwd( std::vector<unsigned>& indexes, unsigned arity) const
    {
        unsigned min = indexes[0] <= indexes[1] ? indexes[0] : indexes[1];
        unsigned max = indexes[0] <= indexes[1] ? indexes[1] : indexes[0];
        return min + max*m_num_fluents_novelty;

    }




    //general version for tuples of arbitrary size
    inline unsigned  tuple2idx_novelty_fwd( std::vector<unsigned>& indexes, unsigned arity) const
    {
        unsigned idx=0;
        unsigned dimension = 1;

        for(int i = arity-1; i >= 0; i--)
        {
            idx += indexes[ i ] * dimension;
            dimension*= m_num_fluents_novelty;
        }

        return idx;

    }
    inline unsigned  tuple2idx_novelty_bwd( std::vector<unsigned>& indexes, unsigned arity) const
    {
        unsigned idx=0;
        unsigned dimension = 1;

        for(int i = arity-1; i >= 0; i--)
        {
            idx += indexes[ i ] * dimension;
            dimension*= m_num_fluents_novelty;
        }

        return idx;

    }

    inline void      idx2tuple_novelty_fwd( std::vector<unsigned>& tuple, Fluent_Vec& fl, unsigned idx, unsigned arity ) const
    {
        unsigned next_idx, div;
        unsigned current_idx = idx;
        unsigned n_atoms = fl.size();

        for(unsigned i = arity-1; i >= 0 ; i--){
            div = aptk::unrolled_pow( n_atoms , i );

            if ( current_idx < div ) {
                next_idx = current_idx;
                current_idx = 0;
            }
            else {
                next_idx = current_idx % div;
                // if current_idx is zero and is the last index, then take next_idx
                current_idx = ( current_idx / div != 0 || i != 0) ? current_idx / div : next_idx;
            }

            tuple[ i ] = fl[ current_idx ];

            current_idx = next_idx;
            if(i == 0) break;
        }
    }
    inline void      idx2tuple_novelty_bwd( std::vector<unsigned>& tuple, Fluent_Vec& fl, unsigned idx, unsigned arity ) const
    {
        unsigned next_idx, div;
        unsigned current_idx = idx;
        unsigned n_atoms = fl.size();

        for(unsigned i = arity-1; i >= 0 ; i--){
            div = aptk::unrolled_pow( n_atoms , i );

            if ( current_idx < div ) {
                next_idx = current_idx;
                current_idx = 0;
            }
            else {
                next_idx = current_idx % div;
                // if current_idx is zero and is the last index, then take next_idx
                current_idx = ( current_idx / div != 0 || i != 0) ? current_idx / div : next_idx;
            }

            tuple[ i ] = fl[ current_idx ];

            current_idx = next_idx;
            if(i == 0) break;
        }
    }
    //const STRIPS_Problem&	m_strips_model;

    std::vector<Search_Node*>     m_nodes_tuples_novelty_fwd;
    std::vector<Search_Node*>     m_nodes_tuples_novelty_bwd;
    unsigned                      m_arity_novelty;
    unsigned long                 m_num_tuples_novelty;
    unsigned                      m_num_fluents_novelty;
    bool			      m_verbose_novelty;


    const STRIPS_Problem&	m_strips_model_bwd;
    const STRIPS_Problem&	m_strips_model_fwd;
   // std::vector< std::vector<Search_Node*> >     m_nodes_tuples_by_partition;
	std::vector< std::vector<Search_Node*> >     m_nodes_tuples_by_partition_bwd;
    std::vector< std::vector<Search_Node*> >     m_nodes_tuples_by_partition_fwd;
  //      unsigned                m_arity;
    unsigned                    m_arity_bwd;
    unsigned                    m_arity_fwd;

	unsigned long           m_num_tuples_bwd;
    unsigned long           m_num_tuples_fwd;

	unsigned                m_num_fluents_bwd;
    unsigned                m_num_fluents_fwd;

	//unsigned                m_max_memory_size_MB;
    unsigned                m_max_memory_size_MB_bwd;
    unsigned                m_max_memory_size_MB_fwd;

	//bool                    m_always_full_state;
	bool                    m_always_full_state_bwd;
	bool                    m_always_full_state_fwd;

	unsigned                m_partition_size_bwd;
    unsigned                m_partition_size_fwd;

	bool			m_verbose_bwd;
	bool            m_verbose_fwd;

	//bool is_bwd = true;
};


}

}

#endif // novelty_partition.hxx
