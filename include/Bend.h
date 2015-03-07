// -----------------------------------------------------------------------------
// libDDG -- Bend.h
// -----------------------------------------------------------------------------
//
// Bend Constraint to achieve correct cloth dynamics between adjacent triangles
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_BEND_H
#define DDG_BEND_H

#include "Face.h"
#include "HalfEdge.h"
#include "Vector.h"
#include "Mesh.h"
#include "Constraint.h"

namespace DDG
{

   class Bend : public Constraint
   {
      public:

         Bend( Mesh& surface_ptr )
         {
            m_equality = true;
            m_cardinality = 6;
            
         };

         virtual ~Bend( void );

         // satisfied if m_equality ? forceFunction(positions) = 0 : forceFunction(positions) >= 0;
         virtual double forceFunction( vector< Vector* > positions );

         // same as forceFunction but for external forces and other forces only dependent on single vertex
         virtual double forceFunction( Vector& position );

      protected:
      
      private:



   };
}

#endif

