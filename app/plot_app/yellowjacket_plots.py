""" This contains the list of all drawn plots on the yellowjacket page """

from html import escape

from bokeh.layouts import column
from bokeh.models import Range1d
from bokeh.models.widgets import Button
from bokeh.io import curdoc

from config import *
from helper import *
from leaflet import ulog_to_polyline
from plotting import *
from plotted_tables import (
    get_logged_messages, get_changed_parameters,
    get_info_table_html, get_heading_html, get_error_labels_html,
    get_hardfault_html, get_corrupt_log_html
    )

#pylint: disable=cell-var-from-loop, undefined-loop-variable,
#pylint: disable=consider-using-enumerate,too-many-statements


def get_yellowjacket_plots(ulog, px4_ulog, db_data, vehicle_data, link_to_main_plots):
    """
    Create list of custom yellowjacket bokeh plots to show
    """

    page_intro = """
<p>
This page shows custom graphs generated for the DarkHive Yellowjacket platform.
</p>
    """
    curdoc().template_variables['title_html'] = get_heading_html(
        ulog, px4_ulog, db_data, None, [('Open Main Plots', link_to_main_plots)],
        'Yellowjacket Specific') + page_intro

    plots = []
    data = ulog.data_list
    
    # COMPATIBILITY support for old logs
    if any(elem.name in ('vehicle_air_data', 'vehicle_magnetometer') for elem in data):
        baro_alt_meter_topic = 'vehicle_air_data'
        magnetometer_ga_topic = 'vehicle_magnetometer'
    else: # old
        baro_alt_meter_topic = 'sensor_combined'
        magnetometer_ga_topic = 'sensor_combined'
    manual_control_sp_controls = ['roll', 'pitch', 'yaw', 'throttle']
    manual_control_sp_throttle_range = '[-1, 1]'
    for topic in data:
        if topic.name == 'system_power':
            # COMPATIBILITY: rename fields to new format
            if 'voltage5V_v' in topic.data:     # old (prior to PX4/Firmware:213aa93)
                topic.data['voltage5v_v'] = topic.data.pop('voltage5V_v')
            if 'voltage3V3_v' in topic.data:    # old (prior to PX4/Firmware:213aa93)
                topic.data['sensors3v3[0]'] = topic.data.pop('voltage3V3_v')
            if 'voltage3v3_v' in topic.data:
                topic.data['sensors3v3[0]'] = topic.data.pop('voltage3v3_v')
        if topic.name == 'tecs_status':
            if 'airspeed_sp' in topic.data: # old (prior to PX4-Autopilot/pull/16585)
                topic.data['true_airspeed_sp'] = topic.data.pop('airspeed_sp')
        if topic.name == 'manual_control_setpoint':
            if 'throttle' not in topic.data: # old (prior to PX4-Autopilot/pull/15949)
                manual_control_sp_controls = ['y', 'x', 'r', 'z']
                manual_control_sp_throttle_range = '[0, 1]'

    if any(elem.name == 'vehicle_angular_velocity' for elem in data):
        rate_estimated_topic_name = 'vehicle_angular_velocity'
        rate_groundtruth_topic_name = 'vehicle_angular_velocity_groundtruth'
        rate_field_names = ['xyz[0]', 'xyz[1]', 'xyz[2]']
    else: # old
        rate_estimated_topic_name = 'vehicle_attitude'
        rate_groundtruth_topic_name = 'vehicle_attitude_groundtruth'
        rate_field_names = ['rollspeed', 'pitchspeed', 'yawspeed']
    if any(elem.name == 'manual_control_switches' for elem in data):
        manual_control_switches_topic = 'manual_control_switches'
    else: # old
        manual_control_switches_topic = 'manual_control_setpoint'
    dynamic_control_alloc = any(elem.name in ('actuator_motors', 'actuator_servos')
                                for elem in data)
    actuator_controls_0 = ActuatorControls(ulog, dynamic_control_alloc, 0)
    actuator_controls_1 = ActuatorControls(ulog, dynamic_control_alloc, 1)

    # initialize flight mode changes
    flight_mode_changes = get_flight_mode_changes(ulog)
    
    # initialize parameter changes
    changed_params = None
    if not 'replay' in ulog.msg_info_dict: # replay can have many param changes
        if len(ulog.changed_parameters) > 0:
            changed_params = ulog.changed_parameters
            plots.append(None) # save space for the param change button
    
    ### Add all data plots ###
    
    x_range_offset = (ulog.last_timestamp - ulog.start_timestamp) * 0.05
    x_range = Range1d(ulog.start_timestamp - x_range_offset, ulog.last_timestamp + x_range_offset)
 
    
    # Additional Altitude Sources
    data_plot = DataPlot(data, plot_config, 'distance_sensor',
                         y_axis_label = '[m]', title = 'Altitudes',
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph(['current_distance'], ['#ae1717'], ['dist_sensor'])
    data_plot.change_dataset(baro_alt_meter_topic)
    data_plot.add_graph(['baro_alt_meter'], ['#03cafc'], ['baro'])
    data_plot.change_dataset('estimator_local_position')
    data_plot.add_graph([lambda data: ('z', data['z']*-1.0), 'dist_bottom'], ['#90fc03','#03fc4e'], ['estimator_z', 'estimator_dist_bottom'])
    data_plot.change_dataset('vehicle_local_position')
    data_plot.add_graph([lambda data: ('z', data['z']*-1.0)], ['#fce303'], ['local_pos_z'])
    data_plot.change_dataset('vehicle_local_position_setpoint')
    data_plot.add_graph([lambda data: ('z', data['z']*-1.0)], ['#fc03f8'], ['local_pos_setpoint_z'])
    data_plot.change_dataset('vehicle_visual_odometry')
    data_plot.add_graph([lambda data: ('z', data['z']*-1.0)], ['#fc5603'], ['visual_z'])
    plot_flight_modes_background(data_plot, flight_mode_changes)
    

    if data_plot.finalize() is not None: plots.append(data_plot)
    
    # X&Y Velocities    
    data_plot = DataPlot(data, plot_config, 'estimator_local_position',
                         y_axis_label = '[m]', title = 'X&Y Velocities',
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph(['vx', 'vy', 'vxy_reset_counter'], colors3, ['local_pos_vx', 'local_pos_vy', 'vxy_reset'])
    data_plot.change_dataset('estimator_optical_flow_vel')
    data_plot.add_graph(['vel_ne[0]', 'vel_ne[1]'], ['#03cafc', '#ca03fc'], ['flow_ne_x', 'flow_ne_y'])
    plot_flight_modes_background(data_plot, flight_mode_changes)
    
    if data_plot.finalize() is not None: plots.append(data_plot)
    
    # Extended VIO message
    data_plot = DataPlot(data, plot_config, 'vehicle_visual_odometry_extended',
                         y_axis_label = '', title = 'VIO Extended',
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph(['state', 'n_total_features', 'error_code'], colors3, ['state', 'n_features', 'err_code'])
    data_plot.change_dataset('vehicle_visual_odometry')
    data_plot.add_graph(['quality'], ['#03cafc'], ['quality'])
    plot_flight_modes_background(data_plot, flight_mode_changes)
    
    if data_plot.finalize() is not None: plots.append(data_plot)
    
    # Optical Flow and Body Velocity
    
    data_plot = DataPlot(data, plot_config, 'optical_flow',
                         y_axis_label = '', title = 'Optical Flow',
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph(['quality', 'mode', 'ground_distance_m'], colors3, ['qualilty', 'mode', 'ground distance'])
    plot_flight_modes_background(data_plot, flight_mode_changes)
    
    if data_plot.finalize() is not None: plots.append(data_plot)
    
    # Estimator Warning Events
    try:
        data_plot = DataPlot(data, plot_config, 'estimator_event_flags',
                             y_start=0, title='Estimator Warning Events',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        estimator_event_flags = ulog.get_dataset('estimator_event_flags').data
        plot_data = []
        plot_labels = []
        input_data = [
            ('Warning Events', estimator_event_flags['warning_event_changes']),
            ('Hgt sensor timeout', estimator_event_flags['height_sensor_timeout']),
            ('Stopping Navigation', (estimator_event_flags['stopping_navigation'])),
            ('Reset Accel Bias', (estimator_event_flags['invalid_accel_bias_cov_reset'])),
            ('Stopping Mag Use', (estimator_event_flags['stopping_mag_use'])),
            ('Vision Data Stopped', (estimator_event_flags['vision_data_stopped'])),
            ('Yaw Reset (Mag Stopped)', (estimator_event_flags['emergency_yaw_reset_mag_stopped']))
            ]
        # filter: show only the flags that have non-zero samples
        for cur_label, cur_data in input_data:
            if np.amax(cur_data) > 0.1:
                data_label = 'flags_'+str(len(plot_data)) # just some unique string
                plot_data.append(lambda d, data=cur_data, label=data_label: (label, data))
                plot_labels.append(cur_label)
                if len(plot_data) >= 8: # cannot add more than that
                    break

        if len(plot_data) == 0:
            # add the plot even in the absence of any problem, so that the user
            # can validate that (otherwise it's ambiguous: it could be that the
            # estimator_event_flags topic is not logged)
            plot_data = [lambda d: ('flags', input_data[0][1])]
            plot_labels = [input_data[0][0]]
        data_plot.add_graph(plot_data, colors8[0:len(plot_data)], plot_labels)
        if data_plot.finalize() is not None: plots.append(data_plot)
    except (KeyError, IndexError) as error:
        print('Error in estimator plot: '+str(error))
        
    # Estimator Information Events
    try:
        data_plot = DataPlot(data, plot_config, 'estimator_event_flags',
                             y_start=0, title='Estimator Information Events',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        estimator_event_flags = ulog.get_dataset('estimator_event_flags').data
        plot_data = []
        plot_labels = []
        input_data = [
            ('Information Events', estimator_event_flags['information_event_changes']),
            ('Reset vel to flow', estimator_event_flags['reset_vel_to_flow']),
            ('Reset vet to vision', (estimator_event_flags['reset_vel_to_vision'])),
            ('Reset vel to zero', (estimator_event_flags['reset_vel_to_zero'])),
            ('Reset pos to last known', (estimator_event_flags['reset_pos_to_last_known'])),
            ('Reset pos to vision', (estimator_event_flags['reset_pos_to_vision'])),
            ('Starting vision pos fusion', (estimator_event_flags['starting_vision_pos_fusion'])),
            ('Starting vision vel fusion', (estimator_event_flags['starting_vision_vel_fusion'])),
            ('Starting vision yaw fusion', (estimator_event_flags['starting_vision_yaw_fusion']))
            ]
        # filter: show only the flags that have non-zero samples
        for cur_label, cur_data in input_data:
            if np.amax(cur_data) > 0.1:
                data_label = 'flags_'+str(len(plot_data)) # just some unique string
                plot_data.append(lambda d, data=cur_data, label=data_label: (label, data))
                plot_labels.append(cur_label)
                if len(plot_data) >= 8: # cannot add more than that
                    break

        if len(plot_data) == 0:
            # add the plot even in the absence of any problem, so that the user
            # can validate that (otherwise it's ambiguous: it could be that the
            # estimator_event_flags topic is not logged)
            plot_data = [lambda d: ('flags', input_data[0][1])]
            plot_labels = [input_data[0][0]]
        data_plot.add_graph(plot_data, colors8[0:len(plot_data)], plot_labels)
        if data_plot.finalize() is not None: plots.append(data_plot)
    except (KeyError, IndexError) as error:
        print('Error in estimator plot: '+str(error))
    
    # exchange all DataPlot's with the bokeh_plot and handle parameter changes

    param_changes_button = Button(label="Hide Parameter Changes", width=170)
    param_change_labels = []
    # FIXME: this should be a CustomJS callback, not on the server. However this
    # did not work for me.
    def param_changes_button_clicked():
        """ callback to show/hide parameter changes """
        for label in param_change_labels:
            if label.visible:
                param_changes_button.label = 'Show Parameter Changes'
                label.visible = False
                label.text_alpha = 0 # label.visible does not work, so we use this instead
            else:
                param_changes_button.label = 'Hide Parameter Changes'
                label.visible = True
                label.text_alpha = 1
    param_changes_button.on_click(param_changes_button_clicked)


    jinja_plot_data = []
    for i in range(len(plots)):
        if plots[i] is None:
            plots[i] = column(param_changes_button, width=int(plot_width * 0.99))
        if isinstance(plots[i], DataPlot):
            if plots[i].param_change_label is not None:
                param_change_labels.append(plots[i].param_change_label)

            plot_title = plots[i].title
            plots[i] = plots[i].bokeh_plot

            fragment = 'Nav-'+plot_title.replace(' ', '-') \
                .replace('&', '_').replace('(', '').replace(')', '')
            jinja_plot_data.append({
                'model_id': plots[i].ref['id'],
                'fragment': fragment,
                'title': plot_title
                })
    
    return plots
