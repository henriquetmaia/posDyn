// -----------------------------------------------------------------------------
// libDDG -- Distance.h
// -----------------------------------------------------------------------------
//
// Simple Distance Constraint, can also be used as a Stretch Constraint
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_DISTANCE_H
#define DDG_DISTANCE_H


#include "Vector.h"
#include "Vertex.h"
#include "Constraint.h"
#include <math.h>

namespace DDG
{

   class Distance : public Constraint
   {
      public:


         Distance( Mesh* surface, double stiffnessParam = 1. ){
            // version for all edges
            m_cloth = surface;
            for( EdgeIter e = m_cloth->edges.begin(); e != m_cloth->edges.end(); ++e ){
               e->l0 = ( e->he->vertex->position - e->he->flip->vertex->position ).norm();
            }
            m_stiffness = stiffnessParam;

            m_cardinality = m_cloth->edges.size();
         };

/*
         Distance( double& distance, std::vector< std::pair< unsigned, unsigned > > index_pairs, double& stiffnessParam = 1., unsigned& numIters )
         : dist( distance )
         {
            m_indices = index_pairs;
            m_cardinality = 2;
            m_equality = true;
            m_stiffness = stiffnessParam;
            // m_stiffness = pow( 1 - stiffnessParam, 1.0 / numIters );
         };
*/
         Distance( double distance, Vertex* juan, Vertex* dos, double stiffnessParam ){
            dist = distance;
            v1 = juan;
            v2 = dos;
            m_cardinality = 2;
            m_equality = true;
            m_stiffness = stiffnessParam;
         };


         virtual ~Distance( void ){};

         virtual void forceFunction( void ){
            if( m_cardinality != 2 ){
               for( EdgeIter e = m_cloth->edges.begin(); e != m_cloth->edges.end(); ++e ){


                  Vector p1 = e->he->vertex->estimate;
                  Vector p2 = e->he->flip->vertex->estimate;

                  // std::cout << "Ep1e: " << p1 << " p1p: " << e->he->vertex->position << std::endl;
                  // std::cout << "Ep1v: " << e->he->vertex->velocity << std::endl;

                  // std::cout << "Ep2e: " << p2 << " p2p: " << e->he->flip->vertex->position << std::endl;
                  // std::cout << "Ep2v: " << e->he->flip->vertex->velocity << std::endl;



                  Vector n = ( p1 - p2 ) / ( p1 - p2 ).norm();
                  double sumInvMass = e->he->vertex->invMass + e->he->flip->vertex->invMass;
                  if( sumInvMass == 0. ){
                     sumInvMass = 1;
                  }
                  double scaling = ( ( p1 - p2 ).norm() - e->l0 ) / ( sumInvMass );
                  e->he->vertex->estimate -= e->he->vertex->invMass * scaling * n * m_stiffness;
                  e->he->flip->vertex->estimate += e->he->flip->vertex->invMass * scaling * n * m_stiffness;

                  // std::cout << "invMass: " <<  e->he->vertex->invMass  << " scaling: " << scaling << " n: " << n << " m_stiffness: " << m_stiffness << std::endl;
                  // std::cout << "after Ep1e: " << e->he->vertex->estimate << " pDEl: " << e->he->vertex->invMass * scaling * n * m_stiffness << std::endl;
                  
                  

               }
            }
            else{

                  Vector p1 = v1->estimate;
                  Vector p2 = v2->estimate;


                  // std::cout << "Origp1e: " << p1 << std::endl;
                  // std::cout << "Origp2e: " << p2 << std::endl;

                  Vector n = ( p1 - p2 ) / ( p1 - p2 ).norm();

                  double sumInvMass = v1->invMass + v2->invMass;
                  if( sumInvMass == 0. ){
                     sumInvMass = 1;
                  }


                  double scaling = ( ( p1 - p2 ).norm() - dist ) / ( sumInvMass );
                  v1->estimate -= v1->invMass * scaling * n * m_stiffness;
                  v2->estimate += v2->invMass * scaling * n * m_stiffness;

                  std::cout << "delta: " << v1->invMass * scaling * n * m_stiffness << std::endl;
                  std::cout << "invMass: " << v1->invMass << " scaling: " << scaling << " n: " << n << " m_stiffness: " << m_stiffness << std::endl;
                  std::cout << "p1e: " << v1->estimate << " p1p: " << v1->position << std::endl;
                  std::cout << "p1v: " << v1->velocity << std::endl;

                  // std::exit(EXIT_FAILURE);
            }

         }

         virtual Vector forceFunction( Vector& position ){
            std::cerr << "forceFunction ( Vector& position ) not implemented for Constraint:: DISTANCE " << std::endl;
            return Vector();
         }

         virtual void forceFunction( std::vector< Vertex* > vertices, unsigned& numIters )
         {
            Vector n = ( vertices[0]->estimate - vertices[1]->estimate ) / ( vertices[0]->estimate - vertices[1]->estimate ).norm();
            double scaling = ( ( vertices[0]->estimate - vertices[1]->estimate ).norm() - dist ) / ( vertices[0]->invMass + vertices[1]->invMass );
            vertices[0]->estimate -= vertices[0]->invMass * scaling * n * m_stiffness;
            vertices[1]->estimate += vertices[1]->invMass * scaling * n * m_stiffness;
         };

      protected:
      
      private:

         Mesh* m_cloth;
         Vertex* v1;
         Vertex* v2;
         double dist;

   };
}

#endif

