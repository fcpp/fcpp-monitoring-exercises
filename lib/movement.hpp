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

//! @brief Maximum allowed group size.
constexpr int max_group_size = 100;
//! @brief Width of the map.
constexpr int hi_x = 1200;
//! @brief Height of the map.
constexpr int hi_y = 800;

//! @brief Namespace containing the libraries of coordination routines.
namespace coordination {

//! @brief Tags used in the node storage.
namespace tags {
    //! @brief General string that can be used for debugging.
    struct debug {};
    //! @brief Speed of the current node.
    struct speed {};
    //! @brief Offset radius for the current node.
    struct offset {};
}

//! @brief Reaches a target position following streets.
FUN real_t reach_on_streets(ARGS, vec<2> target, real_t max_v, times_t period) { CODE
    constexpr real_t k = 0.75;
    vec<2> v = old(CALL, make_vec(0,0), [&](vec<2> ov){
        return k*ov + node.position() - old(CALL, node.position());
    }) * (1-k);
    target = node.net.closest_space(target);
    vec<2> t = node.net.path_to(node.position(), target);
    if (isnan(t[0]) or isnan(t[1]))
        t = target;
    if (target[0] < 0 or target[1] < 0 or target[0] > hi_x or target[1] > hi_y)
        t = target;
    if (node.position() - t < 0.01)
        t = target;
    if (time_since(CALL, v < 0.1) < 10 and node.current_time() > 50)
        t = target;
    return follow_target(CALL, t, max_v, period);
}
FUN_EXPORT reach_on_streets_t = export_list<vec<2>, time_since_t>;

//! @brief Regulates random movement in groups.
FUN void group_walk(ARGS) { CODE
    using namespace tags;

    vec<2> low = {0, 0};
    vec<2> hi = {hi_x, hi_y};
    times_t period = 1;
    device_t leader = node.uid - (node.uid % max_group_size);
    real_t max_v = node.storage(speed{});
    real_t radius = node.storage(offset{});
    bool first_round = old(CALL, true, false);
    if (node.uid == leader) {
        if (first_round)
            node.position() = node.net.closest_space(node.position());
        // leaders just walk randomly
        vec<2> target = random_rectangle_target(CALL, low, hi);
        old(CALL, target, [&](vec<2> t){
            real_t dist = reach_on_streets(CALL, t, max_v, period);
            return dist > max_v * period ? t : target;
        });
    } else {
        // followers chase the leader up to an offset
        vec<2> t = random_rectangle_target(CALL, make_vec(-radius, -radius), make_vec(radius, radius));
        t = constant(CALL, t) + node.net.node_at(leader).position();
        auto fit_bounds = [](real_t v, real_t mx){
            return max(real_t(0), min(v, mx));
        };
        t = make_vec(fit_bounds(t[0], hi_x), fit_bounds(t[1], hi_y));
        if (first_round)
            node.position() = node.net.closest_space(t); // on the first simulated round
        else
            reach_on_streets(CALL, t, max_v, period); // on following rounds
    }
}
//! @brief Export types used by the group_walk function.
FUN_EXPORT group_walk_t = export_list<rectangle_walk_t<2>, constant_t<vec<2>>, reach_on_streets_t, bool>;

//! @brief Executes a program independently in a partition of the network based on the value of a given key.
GEN(T, G) auto split(ARGS, T&& key, G&& f) { CODE
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
