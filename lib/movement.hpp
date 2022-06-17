// Copyright © 2022 Giorgio Audrito. All Rights Reserved.

/**
 * @file movement.hpp
 * @brief Implementation of the group movement behaviour.
 */

#ifndef FCPP_MOVEMENT_H_
#define FCPP_MOVEMENT_H_

#include "lib/fcpp.hpp"


/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {

const int max_group_size = 100;
const int hi_x = 1200;
const int hi_y = 800;

//! @brief Namespace containing the libraries of coordination routines.
namespace coordination {

//! @brief Tags used in the node storage.
namespace tags {
    //! @brief Speed of the current node.
    struct speed {};
    //! @brief Offset radius for the current node.
    struct offset {};
}

//! @brief Regulates random movement in groups.
FUN void group_walk(ARGS) { CODE
    using namespace tags;

    vec<2> low = {0, 0};
    vec<2> hi = {hi_x, hi_y};
    times_t period = 1;
    device_t leader = node.uid - (node.uid % max_group_size);
    real_t max_v = node.storage(speed{});
    real_t radius = node.storage(offset{});
    if (node.uid == leader) {
        rectangle_walk(CALL, low, hi, max_v, period); // leaders just walk randomly
    } else {
        // followers chase the leader up to an offset
        vec<2> t = random_rectangle_target(CALL, make_vec(-radius, -radius), make_vec(radius, radius));
        t = constant(CALL, t) + node.net.node_at(leader).position();
        if (old(CALL, true, false))
            node.position() = t; // on the first simulated round
        else
            follow_target(CALL, t, max_v, period); // on following rounds
    }
}
//! @brief Export types used by the group_walk function.
FUN_EXPORT group_walk_t = export_list<rectangle_walk_t<2>, constant_t<vec<2>>, bool>;

//! @brief Executes a program independently in a partition of the network based on the value of a given key.
GEN(T, G) auto switcher(ARGS, T&& key, G&& f) { CODE
    internal::trace_key trace_process(node.stack_trace, key);
    return f();
}

}

//! @brief Namespace for component options.
namespace option {
    //! @brief Import tags to be used for component options.
    using namespace component::tags;
    //! @brief Import tags used by aggregate functions.
    using namespace coordination::tags;

    //! @brief Class generating a given arithmetic sequence.
    template <typename R, intmax_t start, intmax_t step>
    using arithmetic_sequence = functor::add<
        functor::acc<distribution::constant_n<R, step>, R>,
        distribution::constant_n<R, start - step>
    >;

    //! @brief The distribution of initial node positions (random in a 1200x800 rectangle).
    using rectangle_d = distribution::rect_n<1, 0, 0, hi_x, hi_y>;

    //! @brief Template asserting a condition on an FCPP option.
    template <bool condition>
    struct option_assert;

    //! @brief Template asserting a condition on an FCPP option (true overload only).
    template <>
    struct option_assert<true> {};

    //! @brief Option generating a group of nodes moving together.
    template <int group_id, int group_size, int group_radius, int group_speed = 0, int start_time = 0>
    DECLARE_OPTIONS(spawn_group,
        option_assert<group_id >= 0>, // group ID should be positive
        option_assert<0 < group_size and group_size < max_group_size>, // group size allowed between 1 and 99
        // group_size spawn events all at start_time
        spawn_schedule<sequence::multiple_n<group_size, start_time>>,
        init<
            uid,    arithmetic_sequence<device_t, max_group_size * group_id, 1>, // arithmetic sequence of device IDs
            x,      rectangle_d, // random displacement of devices in the simulation area
            speed,  distribution::constant_n<double, group_speed * 1000, 3600>, // store the group speed, converting from km/h to m/s
            offset, distribution::constant_n<double, group_radius> // store the group radius
        >
    );
}

}

#endif // FCPP_MOVEMENT_H_
