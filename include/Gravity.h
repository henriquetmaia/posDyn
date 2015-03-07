// -----------------------------------------------------------------------------
// libDDG -- Gravity.h
// -----------------------------------------------------------------------------
//
// Simple Gravity Constraint to achieve an external force
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_GRAVITY_H
#define DDG_GRAVITY_H


#include "Vector.h"
#include "Mesh.h"
#include "Constraint.h"

namespace DDG
{

   class Gravity : public Constraint
   {
      public:

         Gravity( double& accConstant, Vector& direction )
         {
         	direction.normalize();
         	m_dir = direction;
         	m_g = accConstant;
         };

         virtual ~Gravity( void );

         // same as forceFunction but for external forces and other forces only dependent on single vertex
         virtual double forceFunction( Vector& position )
         {
         	return m_g * m_dir;
         };

      protected:
      
      private:

      	// gravitational constant
      	double m_g;

      	// normalized direction vector
      	Vector m_dir;

   };
}

#endif

