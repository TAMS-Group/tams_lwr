/******************************************************************************
                  FRI Utility Library

  Copyright (C) 2012 Walter Fetter Lages <w.fetter@ieee.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

    You can also obtain a copy of the GNU General Public License
    at <http://www.gnu.org/licenses>.

******************************************************************************

2012.10.16	Start of development

******************************************************************************/

#ifndef FRIUTIL_H
#define FRIUTIL_H

/** @file friutil.h
 *	Utility Library for KuKA FRI (FastResearchInterface)
 *	@author Walter Fetter Lages <w.fetter@ieee.org>
 */

/** @defgroup friutil Utility Library for KuKA FRI (FastResearchInterface)
@{
*/

#include "friudp.h"

/** Returns a string describing the controller type.
 *	@param ctrl Controller type.
 */
extern const std::string& controllerStr(FRI_CTRL ctrl);

/** Returns a string describing the connection quality.
 *	@param quality Connection quality.
 */
extern const std::string& qualityStr(FRI_QUALITY quality);

/** Returns a string describing the FRI state.
 *	@param state FRI state.
 */
extern const std::string& stateStr(FRI_STATE state);

/** Returns a string with the representation of a Cartesian frame (homegeneous matrix). The last line is ommited.
 *	@param cartFrm Cartesian frame.
 */
extern std::string cartFrmStr(fri_float_t cartFrm[]);

/** Returns a string with the representation of a vector with LBR_MNJ elements.
 *	LBR_MNJ is the number of managed joints.
 *	@param lbrMnj Vector with LBR_MNJ elements.
 */
extern std::string lbrMnjStr(fri_float_t lbrMnj[]);

/** Returns a string with the representation of a Cartesian vector.
 *	@param cartVec Cartesian vector.
 */
extern std::string cartVecStr(fri_float_t cartVec[]);

/** Returns a string with the representation of a FRI_CART_VEC x LBR_MNJ matrix.
 *	FRI_CART_VEC is the dimension of a Cartesian vector.
 *	LBR_MNJ is the number of managed joints.
 *	FRI_CART_VEC x LBR_MNJ is the dimension of the Jacobian matrix.
 *	@param cartVecXLbrMnj FRI_CART_VEC x LBR_MNJ matrix.
 */
extern std::string cartVecXLbrMnjStr(fri_float_t cartVecXLbrMnj[]);

/** Returns a string with the representation of a LBR_MNJ x LBR_MNJ matrix.
 *	LBR_MNJ is the number of managed joints.
 *	LBR_MNJ x LBR_MNJ is the dimension of the mass matrix.
 *	@param lbrMnjXLbrMnj LBR_MNJ x LBR_MNJ matrix.
 */
extern std::string lbrMnjXLbrMnjStr(fri_float_t lbrMnjXLbrMnj[]);

/** Converts a Cartesian frame to a Cartesian vector.
 *	@param cartFrm Cartesian frame.
 *	@param cartVec Cartesian Vector.
 */
extern void cartFrmToVec(const fri_float_t cartFrm[], fri_float_t cartVec[]);

/** Converts a Cartesian vector to a Cartesian frame.
 *	@param cartVec Cartesian vector.
 *	@param cartFrm Cartesian frame.
 */
extern void cartVecToFrm(const fri_float_t cartVec[], fri_float_t cartFrm[]);

/**
@}
 */

#endif
