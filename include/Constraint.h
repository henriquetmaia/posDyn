// -----------------------------------------------------------------------------
// libDDG -- Constraint.h
// -----------------------------------------------------------------------------
//
// Interface for generalized Position Based Dynamics constraints
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_CONSTRAINT_H
#define DDG_CONSTRAINT_H

#include <vector>
#include <utility>
#include "Vector.h"

namespace DDG
{

   class Constraint
   {
/*
            // needs scaling factor and masses for weighing the results
            for( VertexIter v = m_balloon->vertices.begin(); v != m_balloon->vertices.end(); ++v ){
               double scaling = constraint_func / sum_over_Stencil ( invMass * gradient.norm2() ); 
               v->estimate -= scaling * v->invMass * gradient;
            }
*/

      public:

         virtual ~Constraint( void );
         
         // satisfied if m_equality ? forceFunction(positions) = 0 : forceFunction(positions) >= 0;
         // virtual double forceFunction( std::vector< Vector* > positions ) = 0;

         // same as forceFunction but for external forces and other forces only dependent on single vertex
         // virtual double forceFunction( Vector& position ) = 0;

         // same as forceFunction but for external forces and other forces only dependent on single vertex
         virtual Vector forceFunction( Vector& position ) = 0;

         virtual void forceFunction( void ) = 0;

      protected:

         std::vector< std::pair< unsigned, unsigned > > m_indices; // < Mesh index, Vertex index >

         // Must be between 0 and 1
         double m_stiffness; // TODO: should check/clamp this on entry

         unsigned m_cardinality;

         bool m_equality;

      private:

   };
}

#endif

