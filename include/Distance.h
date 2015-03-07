// -----------------------------------------------------------------------------
// libDDG -- Distance.h
// -----------------------------------------------------------------------------
//
// Simple Distance Constraint
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_DISTANCE_H
#define DDG_DISTANCE_H


#include "Vector.h"
#include "Vertex.h"
#include "Constraint.h"

namespace DDG
{

   class Distance : public Constraint
   {
      public:

         Distance( double& distance, unsigned& p1, unsigned& p2, double& stiffnessParam = 1., unsigned& numIters )
         : dist( distance )
         {
            m_indices.push_back( p1 );
            m_indices.push_back( p2 );
            m_cardinality = 2;
            m_equality = true;

            m_stiffness = pow( 1 - stiffnessParam, 1.0 / numIters );
         };

         virtual ~Distance( void );

         // satisfied if m_equality ? forceFunction(positions) = 0 : forceFunction(positions) >= 0;
         virtual double forceFunction( vector< Vertex* > vertices, unsigned& numIters )
         {
            Vector n = ( vertices[0]->estimate - vertices[1]->estimate ) / ( vertices[0]->estimate - vertices[1]->estimate ).norm();
            double scaling = ( ( vertices[0]->estimate - vertices[1]->estimate ).norm() - dist ) / ( vertices[0]->invMass + vertices[1]->invMass );
            vertices[0]->estimate -= vertices[0]->invMass * scaling * n * m_stiffness;
            vertices[1]->estimate += vertices[1]->invMass * scaling * n * m_stiffness;
         };

      protected:
      
      private:

         double dist;

   };
}

#endif

