#Mo3D mobility simulation framework based on the Mo3 model as defined in:
#L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
#future generation mobile wireless networks", IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541
#and extended in:
#D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments,"
#submitted to Software X, 2026.
import matplotlib.pyplot as plt
import numpy as np
import math
import os

font_dict = {'label_size': 22,
             'ticks_size': 22,
             'legend_size': 20}

# x_data = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
# y_data = [1.746, 1.766, 1.815, 1.961, 2.009, 2.061, 2.138, 2.209, 2.297, 2.373]
# x_label = 'N'
# y_label = 'L [ms]'
# legends = 'r$L_{pay} = 100 byte$, Δf=30 kHz, B=20 MHz, 55 RB'


def plot_curves(x_data, y_data, x_label, y_label, legends,
                marker=None, x_ticks=None, y_ticks=None, colors=None, show_grid=True,
                save_file=None):
    """

    Parameters
    ----------
    x_data : list or numpy array
        Contains the data for the x axis
    y_data : list or numpy array
        Contains data for the y-axis. It can be a list of list, or a 2-dimensional array,
        is more than one curve have to be plotted.
    x_label : str
        Label for the x-axis.
    y_label : str
        Label for the y-axis
    legends : list of str
        Contains the labels for the curves, with the same order of 'y_data'. If one curve
        is passed, 'legends' has to be a list with one element.
    markers : list of char
        Contains the markers for the different curves.
    x_ticks : list, optional
        Contains two list or numpy array elements, one for the tick position, one for the
        values, for the x-axis.
    y_ticks : list, optional
        Contains two list or numpy array elements, one for the tick position, one for the
        values, for the y-axis.
    colors : list, optional
        List with colors for the different curves, with the same order of 'y_data'.
    show_grid : bool, optional
        Show or not the grid.
    save_file : str, optional
        Path + filename to save the plot. If None, it does not save the plot.

    Returns
    -------

    """
    if type(y_data[0]) is list:
        n_runs = len(y_data)
    else:
        n_runs = 1
        y_data = [y_data]

    if type(legends) is str:
        legends = [legends]

    plt.figure(figsize=(12, 8))
    plt.xlabel(x_label, fontsize=font_dict['label_size'])
    plt.ylabel(y_label, fontsize=font_dict['label_size'])
    if x_ticks is not None:
        plt.xticks(x_ticks[0], x_ticks[1], fontsize=font_dict['ticks_size'])
    else:
        plt.xticks(fontsize=font_dict['ticks_size'])
    if y_ticks is not None:
        plt.yticks(y_ticks[0], y_ticks[1], fontsize=font_dict['ticks_size'])
    else:
        plt.yticks(fontsize=font_dict['ticks_size'])
    if show_grid:
        plt.grid('on', alpha=0.5)
    if marker is None:
        marker = [""] * n_runs

    # Plot Data
    for run in range(n_runs):
        if colors is not None:
            plt.plot(x_data, y_data[run], color=colors[run], label=legends[run], marker=marker)
        else:
            plt.plot(x_data, y_data[run], label=legends[run], marker=marker)

    if legends is not None:
        plt.legend(loc="upper center", bbox_to_anchor=(0.5, 1.17), ncol=2, fontsize=font_dict['legend_size'])

    if save_file is not None:
        plt.savefig(save_file, dpi=300)
    plt.show()
    return


def plot_3d_layout(m, trajectory_path=None, layout_path=None):
    if trajectory_path is None:
        if os.path.isdir("/results") and os.access("/results", os.W_OK):
            trajectory_path = "/results/trajectories.html"
        else:
            trajectory_path = os.path.join("results", "trajectories.html")

    if layout_path is None:
        if os.path.isdir("/results") and os.access("/results", os.W_OK):
            layout_path = "/results/layout.html"
        else:
            layout_path = os.path.join(os.path.dirname(trajectory_path), "layout.html")

    import plotly.graph_objects as go
    n_nodes = m.nNodes

    # This figure will hold ONLY the obstacles (used for both plots)
    layout_only = go.Figure()

    # 3D mesh
    for obstacle in m.ObsList:
        obs_type = obstacle[0]

        if obs_type == 1:
            # Parallelepiped obstacle: center (x,y,z), full sizes (dx,dy,dz)
            xmin=obstacle[1]-obstacle[4]/2;
            xmax=obstacle[1]+obstacle[4]/2;
            ymin=obstacle[2]-obstacle[5]/2;
            ymax=obstacle[2]+obstacle[5]/2;
            zmin=obstacle[3]-obstacle[6]/2;
            zmax=obstacle[3]+obstacle[6]/2;
            x = [
                xmin, xmax, xmax, xmin,
                xmin, xmax, xmax, xmin
            ]
            y = [
                ymin, ymin, ymax, ymax,
                ymin, ymin, ymax, ymax
            ]
            z = [
                zmin, zmin, zmin, zmin,
                zmax, zmax, zmax, zmax
            ]

            i = [0, 0, 0, 1, 1, 2, 3, 4, 4, 5, 6, 7]
            j = [1, 2, 3, 2, 5, 3, 7, 5, 6, 6, 7, 4]
            k = [4, 5, 6, 5, 6, 7, 4, 0, 1, 2, 3, 0]

            layout_only.add_trace(go.Mesh3d(x=x,y=y,z=z, color='gray', opacity=1, i=i, j=j, k=k, name='Obstacle'))

        elif obs_type == 2:
            # Elliptic cylinder obstacle: center (x,y,z), semi-axes (a,b), vertical extent dz
            xobs0 = obstacle[1]
            yobs0 = obstacle[2]
            a = obstacle[4]
            b = obstacle[5]
            zmin = obstacle[3] - obstacle[6] / 2
            zmax = obstacle[3] + obstacle[6] / 2

            n_seg = 30
            theta = np.linspace(0, 2 * np.pi, n_seg, endpoint=False)
            x_ring = xobs0 + a * np.cos(theta)
            y_ring = yobs0 + b * np.sin(theta)

            # Vertices: bottom ring, top ring, bottom center, top center
            x = np.concatenate([x_ring, x_ring, [xobs0], [xobs0]])
            y = np.concatenate([y_ring, y_ring, [yobs0], [yobs0]])
            z = np.concatenate([np.full(n_seg, zmin), np.full(n_seg, zmax), [zmin], [zmax]])

            bottom_center = 2 * n_seg
            top_center = 2 * n_seg + 1

            i_faces, j_faces, k_faces = [], [], []
            for s in range(n_seg):
                s_next = (s + 1) % n_seg
                # Side wall: two triangles per segment
                i_faces += [s, s]
                j_faces += [s_next, n_seg + s_next]
                k_faces += [n_seg + s_next, n_seg + s]
                # Bottom cap (fan from bottom_center)
                i_faces.append(bottom_center)
                j_faces.append(s)
                k_faces.append(s_next)
                # Top cap (fan from top_center)
                i_faces.append(top_center)
                j_faces.append(n_seg + s_next)
                k_faces.append(n_seg + s)

            layout_only.add_trace(go.Mesh3d(x=x, y=y, z=z, color='gray', opacity=1,
                                             i=i_faces, j=j_faces, k=k_faces, name='Obstacle'))

        else:
            print(f"Warning: unknown obstacle type {obs_type}, skipping in plot")
            continue

    # Shared layout/scene settings, computed once and reused for both figures
    layout_length = m.x_max - m.x_min
    layout_width = m.y_max - m.y_min
    layout_height = m.z_max - m.z_min
    scene_kwargs = dict(
        scene=dict(
            xaxis=dict(range=[m.x_min, m.x_max]),
            yaxis=dict(range=[m.y_min, m.y_max]),
            zaxis=dict(range=[m.z_min, m.z_max]),
            aspectmode='manual',
            aspectratio=dict(
                x=layout_length / max(layout_length, layout_width, layout_height),
                y=layout_width / max(layout_length, layout_width, layout_height),
                z=layout_height / max(layout_length, layout_width, layout_height)
            )
        ),
        showlegend=True
    )

    # Apply layout settings to the obstacles-only figure and save it
    layout_only.update_layout(title='3D Area Layout', **scene_kwargs)

    if layout_path is not None:
        os.makedirs(os.path.dirname(layout_path), exist_ok=True)
        layout_only.write_html(layout_path)
        print(f"3D layout plot saved to {layout_path}")

    # Build the full figure (obstacles + trajectories) by copying the obstacle traces
    area_layout = go.Figure(data=layout_only.data)

    # UAV trajectories
    colors = ['green', 'blue', 'red', 'purple']  # colori tr
    for i in range(n_nodes):
        area_layout.add_trace(go.Scatter3d(x=m.xPath[:, i], y=m.yPath[:, i], z=m.zPath[:, i], mode='lines',
            line=dict(color=colors[i % len(colors)], width=3),  # Usa colori diversi ciclicamente
            name=f'UAV {i + 1} Trajectory'
        ))
        area_layout.add_trace(go.Scatter3d(
            x=[m.xPath[0, i]], y=[m.yPath[0, i]], z=[m.zPath[0, i]], mode='markers',
            marker=dict(color=colors[i % len(colors)], size=6)
            , name=f'UAV {i + 1} Start'
        ))

    area_layout.update_layout(title='3D Area Layout with UAV Trajectories', **scene_kwargs)

    if trajectory_path is not None:
        os.makedirs(os.path.dirname(trajectory_path), exist_ok=True)
        area_layout.write_html(trajectory_path)
        print(f"3D trajectory plot saved to {trajectory_path}")
    else:
        area_layout.show()

def plot_obstacle_avoidance(k, x, y, prev_theta, theta, forbiddenRange, mergedforbiddenRange):
    # Plot visualization of adjustments
    fig, ax = plt.subplots()
    circle = plt.Circle((x, y), 1, color='b', fill=False, linestyle='dashed')
    ax.add_patch(circle)

    first_label = True
    for l in range(len(forbiddenRange)):
        thetarange = forbiddenRange[l]
        if thetarange != [math.pi, math.pi]:
            theta_vals = np.linspace(thetarange[0], thetarange[1], 100)
            x_vals = x + np.cos(theta_vals)
            y_vals = y + np.sin(theta_vals)
            # ax.fill(np.append(x_vals, [x]), np.append(y_vals, [y]), alpha=0.5, label=f'Original Range {l}' if first_label else "")
            # ax.fill(np.append(x_vals, [x]), np.append(y_vals, [y]), alpha=0.5, label=f'Original Range {l//2}')
            ax.fill(np.append(x_vals, [x]), np.append(y_vals, [y]), alpha=0.5)
            # Calcola la posizione centrale del range per posizionare il numero
            mid_theta = (thetarange[0] + thetarange[1]) / 2
            mid_x = x + np.cos(mid_theta)
            mid_y = y + np.sin(mid_theta)

            # Aggiungi il numero del range sulla figura
            ax.text(mid_x, mid_y, f"{l//2}", color='black', fontsize=12, 
                    ha='center', va='center')
            first_label = False

    first_label = True       
    for merged_range in mergedforbiddenRange:
        theta_vals = np.linspace(merged_range[0], merged_range[1], 100)
        x_vals = x + np.cos(theta_vals)
        y_vals = y + np.sin(theta_vals)
        ax.plot(x_vals, y_vals, color='darkred', linewidth=2, linestyle='solid', label=f'Merged Ranges' if first_label else "")
        first_label = False
    
    ax.arrow(x, y, np.cos(prev_theta) * 0.8, np.sin(prev_theta) * 0.8, head_width=0.1, head_length=0.1, fc='r', ec='r', label='Previous Direction')
    if np.cos(prev_theta) != np.cos(theta):
        ax.arrow(x, y, np.cos(theta) * 0.8, np.sin(theta) * 0.8, head_width=0.1, head_length=0.1, fc='g', ec='g', label='New Direction')
    # Plot UAV position
    ax.scatter(x, y, color='blue', marker='o', label=f'UAV {k} Position')

    ax.set_xlim([x - 1.5, x + 1.5])
    ax.set_ylim([y - 1.5, y + 1.5])
    ax.set_aspect('equal')
    ax.axhline(y, color='k', linewidth=0.5)
    ax.axvline(x, color='k', linewidth=0.5)
    ax.legend()
    plt.title("UAV Collision Avoidance - Theta Adjustments")
    plt.show()
