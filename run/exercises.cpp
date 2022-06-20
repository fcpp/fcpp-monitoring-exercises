// Copyright © 2022 Giorgio Audrito. All Rights Reserved.

/**
 * @file exercises.cpp
 * @brief Aggregate computing monitoring exercises.
 */

// [INTRODUCTION]
//! Importing the FCPP library.
#include "lib/fcpp.hpp"
#include "lib/past_ctl.hpp"
#include "lib/slcs.hpp"
#include "lib/movement.hpp"

/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {

//! @brief The maximum communication range between nodes.
constexpr size_t communication_range = 100;

//! @brief Namespace containing the libraries of coordination routines.
namespace coordination {

//! @brief Tags used in the node storage.
namespace tags {
    //! @brief Color of the current node.
    struct node_color {};
    //! @brief Size of the current node.
    struct node_size {};
    //! @brief Shape of the current node.
    struct node_shape {};
    //! @brief Value of the consistency monitor.
    struct consistency {};
    // ... add more as needed, here and in the tuple_store<...> option below
}

// [AGGREGATE PROGRAM]

/**
 * EXERCISES
 *
 * Monitor the following additional properties:
 *
 * 1)    You do not enter a cluster without a previous warning.
 *
 * 2)    You do not enter a cluster without some member of your group having a warning.
 *
 * Every exercise above is designed to help solving the following one.
 */

//! @brief If some node is in cluster alert, it stays alerted until everyone in its group becomes in cluster alert.
FUN bool consistency_monitor(ARGS, bool cluster) { CODE
    using namespace logic;
    // execute independently in different groups
    return split(CALL, node.uid/max_group_size, [&](){
        bool alert_start = Y(CALL, !cluster) & cluster;
        bool alert_end = Y(CALL, cluster) & (!cluster);
        bool all_alerted = G(CALL, cluster);
        bool no_new_alarms_after_all_alerted = AS(CALL, !alert_start, all_alerted);
        // if the alert is ending, there must have been no new alarms after a moment when everyone was alerted
        return alert_end <= no_new_alarms_after_all_alerted;
    });
}
FUN_EXPORT monitor_t = export_list<past_ctl_t, slcs_t>;

//! @brief Main function.
MAIN() {
    using namespace tags;

    // call to the library function handling group-based movement
    group_walk(CALL);

    // compute basic propositions
    bool warning = sum_hood(CALL, mux(node.nbr_dist() < 0.25*communication_range, 1, 0)) > 5; // more than 5 neighbours within 25m?
    bool cluster = sum_hood(CALL, mux(warning, 1, 0)) >= 3; // at least 3 neighbours also on "warning"?

    // sample logic formula
    bool monitor_result = consistency_monitor(CALL, cluster);
    node.storage(consistency{}) = monitor_result;

    // display formula values in the user interface
    node.storage(node_size{}) = cluster ? 20 : 10;
    node.storage(node_color{}) = color(monitor_result ? GREEN : RED);
    node.storage(node_shape{}) = warning ? shape::star : shape::sphere;
}
//! @brief Export types used by the main function (update it when expanding the program).
FUN_EXPORT main_t = export_list<group_walk_t, monitor_t>;

} // namespace coordination

// [SYSTEM SETUP]

//! @brief Namespace for component options.
namespace option {

//! @brief Import tags to be used for component options.
using namespace component::tags;
//! @brief Import tags used by aggregate functions.
using namespace coordination::tags;

//! @brief Description of the round schedule.
using round_s = sequence::periodic<
    distribution::interval_n<times_t, 0, 1>,    // uniform time in the [0,1] interval for start
    distribution::weibull_n<times_t, 10, 1, 10> // weibull-distributed time for interval (10/10=1 mean, 1/10=0.1 deviation)
>;
//! @brief The sequence of network snapshots (one every simulated second).
using log_s = sequence::periodic_n<1, 0, 1>;
//! @brief The contents of the node storage as tags and associated types.
using store_t = tuple_store<
    speed,                      double,
    offset,                     double,
    node_color,                 color,
    node_size,                  double,
    node_shape,                 shape,
    consistency,                bool,
    debug,                      std::string
>;
//! @brief The tags and corresponding aggregators to be logged (change as needed).
using aggregator_t = aggregators<
    consistency,                aggregator::mean<double>
>;

//! @brief Plot description.
using plotter_t = plot::split<plot::time, plot::values<aggregator_t, common::type_sequence<>, consistency>>;

//! @brief The general simulation options.
DECLARE_OPTIONS(list,
    parallel<true>,      // multithreading enabled on node rounds
    synchronised<false>, // optimise for asynchronous networks
    program<coordination::main>,   // program to be run (refers to MAIN above)
    exports<coordination::main_t>, // export type list (types used in messages)
    retain<metric::retain<3,1>>,   // messages are kept for 3 seconds before expiring
    round_schedule<round_s>, // the sequence generator for round events on nodes
    log_schedule<log_s>,     // the sequence generator for log events on the network
    store_t,       // the contents of the node storage
    aggregator_t,  // the tags and corresponding aggregators to be logged
    plot_type<plotter_t>, // the plot description
    area<0, 0, hi_x, hi_y>, // bounding coordinates of the simulated space
    connector<connect::fixed<communication_range>>, // connection allowed within a fixed comm range
    shape_tag<node_shape>, // the shape of a node is read from this tag in the store
    size_tag<node_size>,   // the size  of a node is read from this tag in the store
    color_tag<node_color>, // the color of a node is read from this tag in the store
    // group-id, number of nodes in group, radius, speed:
    spawn_group<0, 1,  0,   20>, // group 0: a single node biking
    spawn_group<1, 20, 50,  3>,  // group 1: a large group strolling
    spawn_group<2, 10, 20,  5>,  // group 2: a medium sized, tightly packed group walking
    spawn_group<3, 10, 80,  5>,  // group 3: a medium sized, loosely packed group walking
    spawn_group<4, 40, 200, 10>
    // add groups as you wish
    /**
     * realistic urban speeds:
     * - standing:  0 km/h
     * - strolling: 3 km/h
     * - walking:   5 km/h
     * - running:  10 km/h
     * - biking:   20 km/h
     * - slow car: 30 km/h
     * - fast car: 50 km/h
     * - drone:    80 km/h
     */
);

} // namespace option

std::ostream& operator<<(std::ostream& o, map_navigator const&) {
    return o;
}

} // namespace fcpp


//! @brief The main function.
int main() {
    using namespace fcpp;

    //! @brief The network object type (interactive simulator with given options).
    using net_t = component::interactive_simulator<option::list>::net;
    //! @brief Create the navigator from the obstacles map.
    map_navigator obj = map_navigator("obstacles.png"); // remove argument for ignoring obstacles
    //! @brief Create the plotter object.
    option::plotter_t p;
    //! @brief The initialisation values (simulation name).
    auto init_v = common::make_tagged_tuple<option::name, option::texture, option::map_navigator_obj, option::plotter>("Monitoring Exercises", "map.jpg", obj, &p);
    std::cout << "/*\n"; // avoid simulation output to interfere with plotting output
    {
        //! @brief Construct the network object.
        net_t network{init_v};
        //! @brief Run the simulation until exit.
        network.run();
    }
    std::cout << "*/\n"; // avoid simulation output to interfere with plotting output
    std::cout << plot::file("exercises", p.build()); // write plots
    return 0;
}
