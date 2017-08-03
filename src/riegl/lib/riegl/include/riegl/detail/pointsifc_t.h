/* $Id: pointsifc_t.h 428 2010-05-10 08:44:04Z rs $ */

#ifndef POINTSIFC_T_H
#define POINTSIFC_T_H

#include <riegl/detail/baseifc_t.h>

/*!\file
 * Compound data types used in the DLL interface.
 */

#ifdef __cplusplus
extern "C"
{
#endif

/*!\brief a point in cartesian coordinates */
typedef struct scanifc_xyz32_t
{
    /*!\brief x coordinates */
    scanifc_float32_t x;

    /*!\brief y coordinates */
    scanifc_float32_t y;

    /*!\brief z coordinates */
    scanifc_float32_t z;

} scanifc_xyz32;

/*!\brief per point attributes */
typedef struct scanifc_attributes_t
{
    /*!\brief relative amplitude in [dB] */
    scanifc_float32_t amplitude;

    /*!\brief relative reflectance in [dB] */
    /*! The relative reflectance is the received power relative
        to the power that would be received from a white diffuse
        target at the same distance. The surface normal of this
        target is assumed to be in parallel to the laser beam
        direction.
     */
    scanifc_float32_t reflectance;

    /*!\brief a measure of pulse shape distortion.*/
    scanifc_uint16_t  deviation;

    /*!\brief some bit-mapped attribute values*/
    /*! The flags combine some attributes into a single value for
        compact storage.
        \verbatim
        bit1-0: 0 .. single echo
                1 .. first echo
                2 .. interior echo
                3 .. last echo
        bit2: 1 .. high power
        bit3: 1 .. rising edge
        bit4: 1 .. pseudo echo with fixed range 0.1m
        bit5: 1 .. sw calculated target
        bit6: 1 .. pps not older than 1.5 sec
        bit7: 1 .. time in pps timeframe
        bit8-15: reserved
        \endverbatim
     */
    scanifc_uint16_t  flags;

    /*!\brief background radiation. */
    scanifc_float32_t background_radiation;

} scanifc_attributes;

/*!\brief time stamp in [ns] */
/*! The epoch (i.e. where time_ns eqals zero) depends on a flag
    that has to be supplied in the open call.
 */
typedef scanifc_uint64_t scanifc_time_ns;

#ifdef __cplusplus
}
#endif

#endif /* POINTSIFC_T_H */
