// -----------------------------------------------------------------------------
// libDDG -- Constraint.h
// -----------------------------------------------------------------------------
//
// Interface for generalized Position Based Dynamics constraints
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_CONSTRAINT_H
#define DDG_CONSTRAINT_H

#include "Face.h"
#include "HalfEdge.h"
#include "Vector.h"
#include "Mesh.h"

namespace DDG
{

   class Constraint
   {
      public:

         virtual ~Constraint( void );
         
         // satisfied if m_equality ? forceFunction(positions) = 0 : forceFunction(positions) >= 0;
         virtual double forceFunction( std::vector< Vector* > positions ) = 0;

         // same as forceFunction but for external forces and other forces only dependent on single vertex
         virtual double forceFunction( Vector& position ) = 0;

      protected:

      private:

         std::vector< unsigned > m_indices;

         // Must be between 0 and 1
         double m_stiffness;

         unsigned m_cardinality;

         bool m_equality;

   };
}

#endif

