
import pyglet
from pyglet.window import key as pyglet_key
import rclpy
import rclpy.node
import rclpy.executors
import py_trees_ros_interfaces.msg
import contextlib
import math
import threading
import typing as tp
import uuid
import unique_identifier_msgs.msg
import queue
import time


def rounded_rectangle_vertex_list(x, y, width, height, radius, radius_segments):
    border = []
    start_angles = [i * math.pi/2 for i in range(4)]
    centers = [
        (x + width - radius, y + height - radius),
        (x + radius, y + height - radius),
        (x + radius, y + radius),
        (x + width - radius, y + radius),
    ]
    for start_angle, center in zip(start_angles, centers):
        for i in range(radius_segments):
            angle = i / (radius_segments-1) * math.pi / 2 + start_angle
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            border.append([x, y])
    return border, centers


class BorderedRoundedAccentedRectangle(pyglet.shapes.ShapeBase):

    def __init__(self, x, y, width, height,
                 border_thickness=3, border_radius=15, border_radius_segments=5,
                 alpha=255, color=(255, 255, 255), border_color=(100, 100, 100),
                 accent_color=(180, 180, 180), accent_radius=5, accent_radius_segments=10,
                 batch=None, group=None):
        self._x = x
        self._y = y
        self._width = width
        self._height = height

        self._rgba = *color, alpha
        self._border_rgba = *border_color, alpha
        self._accent_rgba = *accent_color, 255

        program = pyglet.shapes.get_default_shader()
        self._batch = batch or pyglet.graphics.Batch()
        self._group = self.group_class(
            pyglet.gl.GL_SRC_ALPHA, pyglet.gl.GL_ONE_MINUS_SRC_ALPHA, program, group)

        self._eborder, _ = rounded_rectangle_vertex_list(
            self._x, self._y, self._width, self._height,
            border_radius, border_radius_segments)
        self._iborder, _ = rounded_rectangle_vertex_list(
            self._x + border_thickness, self._y + border_thickness,
            self._width - 2*border_thickness, self._height - 2*border_thickness,
            border_radius - border_thickness, border_radius_segments)
        self._aborder, _ = rounded_rectangle_vertex_list(
            self._x + border_thickness, self._y + border_thickness,
            2*border_radius, self._height - 2*border_thickness,
            border_radius - border_thickness, border_radius_segments)
        self.nb = len(self._eborder)

        self._num_verts = len(self._get_vertices()) // 2

        self._create_vertex_list()

    def __contains__(self, point):
        return pyglet.shapes.Box.__contains__(self, point)

    def _create_vertex_list(self):

        nb = self.nb

        indices = []

        # connect interior and exterior border
        for i in range(nb):
            indices.extend([i, (i+1) % nb, (i+1) % nb+nb, i, (i+1) % nb+nb, i+nb])

        # fill interior border
        for i in range(nb-1):
            indices.extend([2*nb+i, 2*nb+(i+1) % nb, 2*nb+(nb-1)])

        # fill accent border
        for i in range(nb-1):
            indices.extend([3*nb+i, 3*nb+(i+1) % nb, 3*nb+(nb-1)])

        self._vertex_list = self._group.program.vertex_list_indexed(
            self._num_verts, self._draw_mode, indices, self._batch, self._group,
            position=('f', self._get_vertices()),
            colors=('Bn', self._get_colors()),
            translation=('f', (self._x, self._y) * self._num_verts))

    def _get_vertices(self):
        iborder = sum([[x - self._anchor_x, y - self._anchor_y] for x, y in self._iborder], [])
        eborder = sum([[x - self._anchor_x, y - self._anchor_y] for x, y in self._eborder], [])
        aborder = sum([[x - self._anchor_x, y - self._anchor_y] for x, y in self._aborder], [])
        return iborder + eborder + iborder + aborder

    def _update_vertices(self):
        self._vertex_list.position[:] = self._get_vertices()

    def _get_colors(self):
        nb = self.nb
        return self._border_rgba * 2*nb + self._rgba * nb + self._accent_rgba * nb

    def _update_color(self):
        self._vertex_list.colors[:] = self._get_colors()

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, value):
        self._width = value
        self._update_vertices()

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, value):
        self._height = value
        self._update_vertices()

    @property
    def border_color(self):
        return self._border_rgba

    @border_color.setter
    def border_color(self, values):
        r, g, b, *a = values
        if a:
            alpha = a[0]
        else:
            alpha = self._rgba[3]
        self._border_rgba = r, g, b, alpha
        self._rgba = *self._rgba[:3], alpha
        self._accent_rgba = *self._accent_rgba[:3], alpha
        self._update_color()

    @property
    def color(self):
        return self._rgba

    @color.setter
    def color(self, values):
        r, g, b, *a = values
        if a:
            alpha = a[0]
        else:
            alpha = self._rgba[3]
        self._rgba = r, g, b, alpha
        self._border_rgba = *self._border_rgba[:3], alpha
        self._accent_rgba = *self._accent_rgba[:3], alpha
        self._update_color()

    @property
    def accent_color(self):
        return self._accent_rgba

    @color.setter
    def accent_color(self, values):
        r, g, b, *a = values
        if a:
            alpha = a[0]
        else:
            alpha = self._rgba[3]
        self._accent_rgba = r, g, b, alpha
        self._rgba = *self._rgba[:3], alpha
        self._border_rgba = *self._border_rgba[:3], alpha
        self._update_color()

def msg_to_uuid4(msg: unique_identifier_msgs.msg.UUID) -> uuid.UUID:
    return uuid.UUID(bytes=bytes(msg.uuid), version=4)


def reorder(
        uuid_to_behaviour: tp.Dict[uuid.UUID, py_trees_ros_interfaces.msg.Behaviour],
        root_id: uuid.UUID,
        box_shapes: tp.Dict[uuid.UUID, BorderedRoundedAccentedRectangle],
        bezier_curves: tp.Dict[uuid.UUID, pyglet.shapes.BezierCurve],
        texts: tp.Dict[uuid.UUID, pyglet.text.Label],
):
    # https://github.com/BehaviorTree/Groot/blob/66aee170c67517849b0ae5084e7cec01995eb349/bt_editor/utils.cpp#L89

    layer_cursor = []
    nodes_by_level = [[]]

    LEVEL_SPACING = 20
    NODE_SPACING = 5

    def recursive_step(current_layer, behaviour: py_trees_ros_interfaces.msg.Behaviour):
        box_shape = box_shapes[msg_to_uuid4(behaviour.own_id)]

        box_shape.x = layer_cursor[current_layer][0]
        box_shape.y = layer_cursor[current_layer][1]

        nodes_by_level[current_layer].append(behaviour)

        ###

        if len(behaviour.child_ids) == 0:
            return

        recommended_pos = NODE_SPACING * 0.5

        current_layer += 1

        recommended_pos += box_shape.x + box_shape.width/2

        for child_id in behaviour.child_ids:
            child_shape = box_shapes[msg_to_uuid4(child_id)]
            recommended_pos -= (child_shape.width + NODE_SPACING) / 2

        if current_layer >= len(layer_cursor):
            layer_cursor.append([recommended_pos, box_shape.y - LEVEL_SPACING])
            nodes_by_level.append([])
        else:
            recommended_pos = max(recommended_pos, layer_cursor[current_layer][0])
            layer_cursor[current_layer][0] = recommended_pos

        ###

        initial_pos = layer_cursor[current_layer]

        for child_id in behaviour.child_ids:
            child = uuid_to_behaviour[msg_to_uuid4(child_id)]
            child_shape = box_shapes[msg_to_uuid4(child_id)]

            recursive_step(current_layer, child)

            layer_cursor[current_layer][0] += child_shape.width + NODE_SPACING

        ###

        final_pos = layer_cursor[current_layer]

        if len(behaviour.child_ids) > 0:
            first_child_shape = box_shapes[msg_to_uuid4(behaviour.child_ids[0])]
            last_child_shape = box_shapes[msg_to_uuid4(behaviour.child_ids[-1])]
            initial_pos = (first_child_shape.x + first_child_shape.width/2,
                           first_child_shape.y + first_child_shape.height/2)
            final_pos = (last_child_shape.x + last_child_shape.width/2,
                         last_child_shape.y + last_child_shape.height/2)

        ###

        new_x = (final_pos[0] + initial_pos[0]) / 2 - box_shape.width/2
        diff = box_shape.x - new_x

        box_shape.x += -diff
        layer_cursor[current_layer - 1][0] += -diff

    root = uuid_to_behaviour[root_id]

    layer_cursor.append([0.0, 0.0])

    recursive_step(0, root)

    root_shape = box_shapes[msg_to_uuid4(root.own_id)]
    offset_y = -root_shape.height - LEVEL_SPACING
    for i in range(1, len(nodes_by_level)):
        max_height = 0.0
        for node in nodes_by_level[i]:
            box_shape = box_shapes[msg_to_uuid4(node.own_id)]
            box_shape.y = offset_y
            max_height = max(max_height, box_shape.height)
        offset_y += -max_height - LEVEL_SPACING

    ###

    root_box_shape = box_shapes[root_id]

    for own_id in uuid_to_behaviour.keys():
        own_box_shape = box_shapes[own_id]
        own_box_shape.x -= root_box_shape.x

    ###

    for own_id, behaviour in uuid_to_behaviour.items():
        own_box_shape = box_shapes[own_id]
        texts[own_id].x = own_box_shape.x + 13
        texts[own_id].y = own_box_shape.y - 7
        if own_id != root_id:
            parent_box_shape = box_shapes[msg_to_uuid4(behaviour.parent_id)]
            bezier_curves[own_id].points = [
                [parent_box_shape.x, parent_box_shape.y - parent_box_shape.height],
                [parent_box_shape.x, parent_box_shape.y - parent_box_shape.height - LEVEL_SPACING/2],
                [own_box_shape.x, own_box_shape.y + LEVEL_SPACING/2],
                [own_box_shape.x, own_box_shape.y],
            ]


def colorize(
    uuid_to_behaviour: tp.Dict[uuid.UUID, py_trees_ros_interfaces.msg.Behaviour],
    root_id: uuid.UUID,
    box_shapes: tp.Dict[uuid.UUID, BorderedRoundedAccentedRectangle],
    bezier_curves: tp.Dict[uuid.UUID, pyglet.shapes.BezierCurve],
    texts: tp.Dict[uuid.UUID, pyglet.text.Label]
):

    for own_id, behaviour in uuid_to_behaviour.items():

        status_color = (100, 100, 100)
        type_color = (150, 150, 150)
        active_alpha = 255 if behaviour.is_active else 80

        if behaviour.status == py_trees_ros_interfaces.msg.Behaviour.SUCCESS:
            status_color = (47, 227, 47)
        elif behaviour.status == py_trees_ros_interfaces.msg.Behaviour.FAILURE:
            status_color = (255, 46, 46)
        elif behaviour.status == py_trees_ros_interfaces.msg.Behaviour.RUNNING:
            status_color = (255, 255, 255)

        if behaviour.type == py_trees_ros_interfaces.msg.Behaviour.BEHAVIOUR:
            type_color = (180, 180, 180)
        elif behaviour.type == py_trees_ros_interfaces.msg.Behaviour.SEQUENCE:
            type_color = (255, 165, 0)
        elif behaviour.type == py_trees_ros_interfaces.msg.Behaviour.SELECTOR:
            type_color = (0, 255, 255)
        elif behaviour.type == py_trees_ros_interfaces.msg.Behaviour.PARALLEL:
            type_color = (255, 255, 0)
        elif behaviour.type == py_trees_ros_interfaces.msg.Behaviour.CHOOSER:
            type_color = (0, 255, 255)
        elif behaviour.type == py_trees_ros_interfaces.msg.Behaviour.DECORATOR:
            type_color = (255, 255, 255)

        # doing these checks on the cpu seems more performant than always uploading
        # new color information to the GPU shader

        if box_shapes[own_id].border_color != (*status_color, active_alpha):
            box_shapes[own_id].border_color = *status_color, active_alpha

        if own_id != root_id:
            if bezier_curves[own_id].color != (*status_color, active_alpha):
                bezier_curves[own_id].color = *status_color, active_alpha

        if box_shapes[own_id].color != (*type_color, active_alpha):
            box_shapes[own_id].accent_color = *type_color, active_alpha

        if texts[own_id].color != (*type_color, active_alpha):
            texts[own_id].color = *type_color, active_alpha

@contextlib.contextmanager
def transform(window, x=0.0, y=0.0, angle=0.0, scale=1.0):
    original_view = window.view.translate((0.0, 0.0, 0.0))
    try:
        if x != 0.0 or y != 0.0:
            window.view = window.view.translate((x, y, 0.0))
        if angle != 0.0:
            window.view = window.view.rotate((180./math.pi*angle, 0.0, 0.0, 1.0))
        if scale != 1.0:
            window.view = window.view.scale((scale, scale, 1.0))
        yield None
    finally:
        window.view = original_view


def main():
    rclpy.init()
    node = rclpy.node.Node("pytrees_viewer", parameter_overrides=[])
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    antialiasing = node.declare_parameter("antialiasing", False).value

    window_args = { "resizable": True }
    if antialiasing:
        window_args["config"] = pyglet.gl.Config(sample_buffers=1, samples=4)

    window = pyglet.window.Window(**window_args)

    behaviour_tree_msg_queue = queue.Queue()

    keys = pyglet_key.KeyStateHandler()
    window.push_handlers(keys)

    pan_x = window.width/2
    pan_y = window.height-50
    zoom = 1.0

    batch0 = pyglet.graphics.Batch()
    batch1 = pyglet.graphics.Batch()
    batch2 = pyglet.graphics.Batch()

    box_shapes: tp.Dict[uuid.UUID, BorderedRoundedAccentedRectangle] = dict({})

    bezier_curves: tp.Dict[uuid.UUID, pyglet.shapes.BezierCurve] = dict({})

    texts: tp.Dict[uuid.UUID, pyglet.text.Label] = dict({})

    fps_display = pyglet.window.FPSDisplay(window)
    fps_display.label.font_size = 12

    update_ms_text = pyglet.text.Label(
        text="0 ms", font_size=12, x=70, y=10, dpi=96,
        color=(127, 127, 127, 127))

    line_horizontal = pyglet.shapes.Line(
        -10.0, 0.0, 10.0, 0.0, width=1, color=(255, 255, 255, 50), batch=batch0)
    line_vertical = pyglet.shapes.Line(
        0.0, -10.0, 0.0, 10.0, width=1, color=(255, 255, 255, 50), batch=batch0)

    def on_draw():
        pyglet.gl.glClearColor(0.094, 0.094, 0.094, 1.0)
        window.clear()
        with transform(window=window, x=pan_x, y=pan_y, scale=zoom):
            with transform(window=window, y=20.0):
                batch0.draw()
            batch1.draw()
            batch2.draw()
        fps_display.draw()
        update_ms_text.draw()

    def on_update(dt: float):
        # if keys[pyglet_key.UP]:
        #     pass

        zero_uuid = msg_to_uuid4(unique_identifier_msgs.msg.UUID(uuid=[0]*16))

        try:
            msg: py_trees_ros_interfaces.msg.BehaviourTree = behaviour_tree_msg_queue.get_nowait()
        except:
            pass
        else:
            ts = time.time()

            uuid_to_behaviour: tp.Dict[uuid.UUID, py_trees_ros_interfaces.msg.Behaviour] = dict({})

            for b in msg.behaviours:
                uuid_to_behaviour[msg_to_uuid4(b.own_id)] = b
                if msg_to_uuid4(b.parent_id) == zero_uuid:
                    root = b
                    root_id = msg_to_uuid4(root.own_id)
            assert root
            assert root_id

            incoming_uuids = set(uuid_to_behaviour.keys())
            existing_uuids = set(box_shapes.keys())
            new_uuids = incoming_uuids - existing_uuids
            dead_uuids = existing_uuids - incoming_uuids

            dead_boxes = [box_shapes.pop(dead) for dead in dead_uuids]
            try:
                dead_curves = [bezier_curves.pop(dead) for dead in dead_uuids]
            except KeyError:
                pass # root node does not have a curve
            dead_texts = [texts.pop(dead) for dead in dead_uuids]

            for new in new_uuids:
                # boxes
                if len(dead_boxes) > 0:
                    box_shapes[new] = dead_boxes.pop()
                else:
                    box_shapes[new] = BorderedRoundedAccentedRectangle(
                        x=0, y=0, width=180, height=60,
                        border_thickness=3, border_radius=10, border_radius_segments=5,
                        color=(40, 40, 40), border_color=(100, 100, 100),
                        batch=batch2
                    )
                    box_shapes[new].anchor_x = box_shapes[new].width / 2
                    box_shapes[new].anchor_y = box_shapes[new].height
                # texts
                if len(dead_texts) > 0:
                    texts[new] = dead_texts.pop()
                    texts[new].text = uuid_to_behaviour[new].name
                else:
                    texts[new] = pyglet.text.Label(
                        uuid_to_behaviour[new].name, font_size=9, x=0, y=0,
                        anchor_x="center", anchor_y="top", dpi=96,
                        color=(255, 255, 255, 255), batch=batch2,
                        width=box_shapes[new].width-34, multiline=True, align="left"
                    )
                    texts[new].set_style("wrap", "char")
                # curves
                if new != root_id:
                    if len(dead_curves) > 0:
                        bezier_curves[new] = dead_curves.pop()
                    else:
                        bezier_curves[new] = pyglet.shapes.BezierCurve(
                            *[[0, 0], [20, 0], [20, 20], [0, 20]], thickness=2, segments=10,
                            color=(255, 0, 0, 255), batch=batch1
                        )

            for dead in dead_boxes:
                dead.delete()

            for dead in dead_curves:
                dead.delete()

            for dead in dead_texts:
                dead.delete()

            if len(new_uuids) > 0 or len(dead_uuids) > 0:
                reorder(uuid_to_behaviour, root_id, box_shapes, bezier_curves, texts)

            if msg.changed or len(new_uuids) > 0 or len(dead_uuids) > 0:
                colorize(uuid_to_behaviour, root_id, box_shapes, bezier_curves, texts)

            update_ms_text.text = f"{int((time.time() - ts) * 1000):d} ms"

    def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
        nonlocal pan_x, pan_y
        if buttons & pyglet.window.mouse.LEFT:
            pan_x += dx
            pan_y += dy

    def on_mouse_scroll(x, y, scroll_x, scroll_y):
        nonlocal zoom, pan_x, pan_y
        delta_zoom = 1.0 + scroll_y * 0.1
        zoom *= delta_zoom
        pan_x = x - (x - pan_x) * delta_zoom
        pan_y = y - (y - pan_y) * delta_zoom

    pyglet.clock.schedule(on_update)
    window.on_draw = on_draw
    window.on_mouse_drag = on_mouse_drag
    window.on_mouse_scroll = on_mouse_scroll

    def behaviour_tree_cb(msg: py_trees_ros_interfaces.msg.BehaviourTree):
        behaviour_tree_msg_queue.put_nowait(msg)

    node.create_subscription(py_trees_ros_interfaces.msg.BehaviourTree,
                             "/bt_node/snapshots", behaviour_tree_cb, 1)

    threading.Thread(target=lambda: executor.spin(), daemon=True).start()

    pyglet.app.run()


if __name__ == "__main__":
    main()
