// -----------------------------------------------------------------------------
// libDDG -- Cloth.h
// -----------------------------------------------------------------------------
//
// Cloth Constraint to achieve correct cloth dynamics
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_CLOTH_H
#define DDG_CLOTH_H

#include "Face.h"
#include "HalfEdge.h"
#include "Vector.h"
#include "Mesh.h"
#include "Constraint.h"

namespace DDG
{

   class Cloth : public Constraint
   {
      public:

         Cloth( Mesh& surface_ptr );

         virtual ~Cloth( void );

         // satisfied if m_equality ? forceFunction(positions) = 0 : forceFunction(positions) >= 0;
         virtual double forceFunction( vector< Vector* > positions );

         // same as forceFunction but for external forces and other forces only dependent on single vertex
         virtual double forceFunction( Vector& position );

      protected:
      
      private:



   };
}

#endif

