// -----------------------------------------------------------------------------
// libDDG -- StaticCollision.h
// -----------------------------------------------------------------------------
//
// Collision Constraint where one (and only one) of the two objects is fixed
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_STATIC_COLLISION_H
#define DDG_STATIC_COLLISION_H


#include "Vector.h"
#include "Vertex.h"
#include "Constraint.h"

namespace DDG
{

   class StaticCollision : public Constraint
   {
      public:

         StaticCollision( void )
         {
            m_stiffness = 1.;
            m_equality = false;

         };

         virtual ~StaticCollision( void );

         virtual double forceFunction( vector< Vertex* > vertices, unsigned& numIters )
         {
            //check if function is less than zero, else skip

            // // or? check if initial position is already inside the object
            // do continous time detection to find collision point
            // check if completely inside the object
            // if so project to surface
         };

      protected:
      
      private:

         Vector m_contactPoint;
         Vector m_contactNormal;

   };
}

#endif

