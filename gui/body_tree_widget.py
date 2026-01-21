"""
Body tree widget for displaying list of bodies and joints
"""
from PySide6.QtWidgets import QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget, QLabel, QMenu
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QAction
from typing import List, Iterable
from core.data_structures import RigidBody, Frame, Joint, Force


class BodyTreeWidget(QWidget):
    """Widget containing tree view of bodies and joints"""
    
    # Signal emitted when a body is selected
    body_selected = Signal(int)  # Emits body ID
    # Signal emitted when a frame is selected
    frame_selected = Signal(str) # Emits frame name
    # Signal emitted when a frame deletion is requested
    delete_frame_requested = Signal(str) # Emits frame name
    # Signal emitted when a joint is selected
    joint_selected = Signal(str) # Emits joint name
    # Signal emitted when a joint deletion is requested
    delete_joint_requested = Signal(str) # Emits joint name
    # Signal emitted when body isolation is requested
    isolate_body_requested = Signal(int) # Emits body ID
    # Signal emitted when exiting isolation mode
    exit_isolation_requested = Signal()
    # Signal emitted when a force is selected
    force_selected = Signal(str) # Emits force name
    # Signal emitted when a force deletion is requested
    delete_force_requested = Signal(str) # Emits force name
    # Signal emitted when a torque is selected
    torque_selected = Signal(str) # Emits torque name
    # Signal emitted when a torque deletion is requested
    delete_torque_requested = Signal(str) # Emits torque name
    # Signal emitted when body deletion is requested
    delete_body_requested = Signal(int) # Emits body ID
    # Signal emitted when multiple bodies deletion is requested
    delete_multiple_bodies_requested = Signal(list) # Emits list of body IDs

    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Create layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Create label for body count
        self.count_label = QLabel("Bodies (0)")
        layout.addWidget(self.count_label)
        
        # Create tree widget
        self.tree = QTreeWidget()
        self.tree.setHeaderLabel("Assembly Structure")
        self.tree.setAlternatingRowColors(True)
        self.tree.setSelectionMode(QTreeWidget.ExtendedSelection)  # Enable multi-selection
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._show_context_menu)
        layout.addWidget(self.tree)
        
        # Initialize groups
        self.bodies_group = QTreeWidgetItem(self.tree)
        self.bodies_group.setText(0, "Bodies")
        self.bodies_group.setExpanded(True)
        
        self.frames_group = QTreeWidgetItem(self.tree)
        self.frames_group.setText(0, "Frames")
        self.frames_group.setExpanded(True)
        
        self.joints_group = QTreeWidgetItem(self.tree)
        self.joints_group.setText(0, "Joints")
        self.joints_group.setExpanded(True)
        
        self.forces_group = QTreeWidgetItem(self.tree)
        self.forces_group.setText(0, "Forces")
        self.forces_group.setExpanded(True)
        
        self.torques_group = QTreeWidgetItem(self.tree)
        self.torques_group.setText(0, "Torques")
        self.torques_group.setExpanded(True)
        
        # Connect signals
        self.tree.itemClicked.connect(self._on_item_clicked)
        
        print("Body tree widget initialized")
    
    def update_bodies(self, bodies: List[RigidBody]):
        """
        Update the tree with a list of bodies
        
        Args:
            bodies: List of RigidBody objects to display
        """
        # Clear existing body items, but keep the group
        self.bodies_group.takeChildren()
        
        # Update count label
        count = len(bodies)
        self.count_label.setText(f"Bodies ({count})")
        
        # Add each body to the tree under bodies group
        for body in bodies:
            item = QTreeWidgetItem(self.bodies_group)
            item.setText(0, body.name)
            # Store body ID as data with user role
            item.setData(0, Qt.UserRole, body.id)
            item.setData(0, Qt.UserRole + 1, "Body") # Type identifier
        
        print(f"Tree widget updated with {count} bodies")

    def update_frames(self, frames: Iterable[Frame]):
        """
        Update the tree with a list of frames

        Args:
            frames: Iterable of Frame objects
        """
        self.frames_group.takeChildren()
        
        count = 0
        for frame in frames:
            item = QTreeWidgetItem(self.frames_group)
            item.setText(0, frame.name)
            item.setData(0, Qt.UserRole, frame.name) # Store name as ID
            item.setData(0, Qt.UserRole + 1, "Frame") # Type identifier
            count += 1
            
        print(f"Tree widget updated with {count} frames")

    def update_joints_list(self, joints: Iterable[Joint]):
        """
        Update the tree with a list of joints

        Args:
            joints: Iterable of Joint objects
        """
        self.joints_group.takeChildren()
        
        count = 0
        for joint in joints:
            item = QTreeWidgetItem(self.joints_group)
            # Add motor indicator if motorized
            display_text = joint.name
            if joint.is_motorized:
                display_text += " ⚡"  # Lightning bolt emoji to indicate motor
            item.setText(0, display_text)
            item.setData(0, Qt.UserRole, joint.name) # Store name as ID
            item.setData(0, Qt.UserRole + 1, "Joint") # Type identifier
            count += 1
            
        print(f"Tree widget updated with {count} joints")
    
    def update_forces_list(self, forces: Iterable[Force]):
        """
        Update the tree with a list of forces

        Args:
            forces: Iterable of Force objects
        """
        self.forces_group.takeChildren()
        
        count = 0
        for force in forces:
            item = QTreeWidgetItem(self.forces_group)
            item.setText(0, force.name)
            item.setData(0, Qt.UserRole, force.name) # Store name as ID
            item.setData(0, Qt.UserRole + 1, "Force") # Type identifier
            count += 1
            
        print(f"Tree widget updated with {count} forces")
    
    def update_torques_list(self, torques: Iterable):
        """
        Update the tree with a list of torques

        Args:
            torques: Iterable of Torque objects
        """
        self.torques_group.takeChildren()
        
        count = 0
        for torque in torques:
            item = QTreeWidgetItem(self.torques_group)
            item.setText(0, torque.name)
            item.setData(0, Qt.UserRole, torque.name) # Store name as ID
            item.setData(0, Qt.UserRole + 1, "Torque") # Type identifier
            count += 1
            
        print(f"Tree widget updated with {count} torques")
    
    def clear(self):
        """Clear all items from the tree"""
        self.bodies_group.takeChildren()
        self.frames_group.takeChildren()
        self.joints_group.takeChildren()
        self.forces_group.takeChildren()
        self.torques_group.takeChildren()
        self.count_label.setText("Bodies (0)")
        print("Tree widget cleared")
    
    def select_body(self, body_id: int):
        """
        Programmatically select a body in the tree by ID
        
        Args:
            body_id: ID of the body to select
        """
        # Find the item with matching body_id in bodies group
        for i in range(self.bodies_group.childCount()):
            item = self.bodies_group.child(i)
            if item.data(0, Qt.UserRole) == body_id:
                # Select the item (this will trigger _on_item_clicked)
                self.tree.setCurrentItem(item)
                # Emit signal
                self.body_selected.emit(body_id)
                break
    
    def select_frame(self, frame_name: str):
        """Programmatically select a frame"""
        for i in range(self.frames_group.childCount()):
            item = self.frames_group.child(i)
            if item.data(0, Qt.UserRole) == frame_name:
                self.tree.setCurrentItem(item)
                self.frame_selected.emit(frame_name)
                break

    def _on_item_clicked(self, item: QTreeWidgetItem, column: int):
        """Handle item click - emit signal with body ID or frame Name"""
        item_type = item.data(0, Qt.UserRole + 1)
        
        if item_type == "Body":
            body_id = item.data(0, Qt.UserRole)
            if body_id is not None:
                print(f"Body selected in tree: {item.text(0)} (ID: {body_id})")
                self.body_selected.emit(body_id)
        elif item_type == "Frame":
            frame_name = item.data(0, Qt.UserRole)
            if frame_name is not None:
                print(f"Frame selected in tree: {frame_name}")
                self.frame_selected.emit(frame_name)
        elif item_type == "Joint":
            joint_name = item.data(0, Qt.UserRole)
            if joint_name is not None:
                print(f"Joint selected in tree: {joint_name}")
                self.joint_selected.emit(joint_name)
        elif item_type == "Force":
            force_name = item.data(0, Qt.UserRole)
            if force_name is not None:
                print(f"Force selected in tree: {force_name}")
                self.force_selected.emit(force_name)
        elif item_type == "Torque":
            torque_name = item.data(0, Qt.UserRole)
            if torque_name is not None:
                print(f"Torque selected in tree: {torque_name}")
                self.torque_selected.emit(torque_name)

    def _show_context_menu(self, position):
        """Show context menu for tree items"""
        item = self.tree.itemAt(position)
        if not item:
            return
            
        item_type = item.data(0, Qt.UserRole + 1)
        
        if item_type == "Body":
            # Get all selected items
            selected_items = self.tree.selectedItems()
            selected_bodies = [item for item in selected_items if item.data(0, Qt.UserRole + 1) == "Body"]
            
            menu = QMenu()
            
            if len(selected_bodies) == 1:
                # Single body selected - show isolation options
                isolate_action = QAction("Isolate Body", self)
                isolate_action.triggered.connect(lambda: self._request_isolate_body(item))
                menu.addAction(isolate_action)
                
                exit_isolate_action = QAction("Exit Isolation", self)
                exit_isolate_action.triggered.connect(self._request_exit_isolation)
                menu.addAction(exit_isolate_action)
                
                menu.addSeparator()
                
                delete_action = QAction("Delete Body", self)
                delete_action.triggered.connect(lambda: self._request_delete_body(item))
                menu.addAction(delete_action)
            else:
                # Multiple bodies selected
                delete_action = QAction(f"Delete {len(selected_bodies)} Bodies", self)
                delete_action.triggered.connect(lambda: self._request_delete_multiple_bodies(selected_bodies))
                menu.addAction(delete_action)
            
            menu.exec(self.tree.viewport().mapToGlobal(position))
        elif item_type == "Frame":
            menu = QMenu()
            delete_action = QAction("Delete Frame", self)
            delete_action.triggered.connect(lambda: self._request_delete_frame(item))
            menu.addAction(delete_action)
            menu.exec(self.tree.viewport().mapToGlobal(position))
        elif item_type == "Joint":
            menu = QMenu()
            delete_action = QAction("Delete Joint", self)
            delete_action.triggered.connect(lambda: self._request_delete_joint(item))
            menu.addAction(delete_action)
            menu.exec(self.tree.viewport().mapToGlobal(position))
        elif item_type == "Force":
            menu = QMenu()
            delete_action = QAction("Delete Force", self)
            delete_action.triggered.connect(lambda: self._request_delete_force(item))
            menu.addAction(delete_action)
            menu.exec(self.tree.viewport().mapToGlobal(position))
        elif item_type == "Torque":
            menu = QMenu()
            delete_action = QAction("Delete Torque", self)
            delete_action.triggered.connect(lambda: self._request_delete_torque(item))
            menu.addAction(delete_action)
            menu.exec(self.tree.viewport().mapToGlobal(position))

    def _request_delete_frame(self, item):
        frame_name = item.data(0, Qt.UserRole)
        if frame_name:
            self.delete_frame_requested.emit(frame_name)

    def _request_delete_joint(self, item):
        joint_name = item.data(0, Qt.UserRole)
        if joint_name:
            self.delete_joint_requested.emit(joint_name)
    
    def _request_delete_force(self, item):
        """Request deletion of a force"""
        force_name = item.data(0, Qt.UserRole)
        if force_name:
            self.delete_force_requested.emit(force_name)
    
    def _request_delete_torque(self, item):
        """Request deletion of a torque"""
        torque_name = item.data(0, Qt.UserRole)
        if torque_name:
            self.delete_torque_requested.emit(torque_name)
    
    def _request_delete_body(self, item):
        """Request deletion of a specific body"""
        body_id = item.data(0, Qt.UserRole)
        if body_id is not None:
            self.delete_body_requested.emit(body_id)
    
    def _request_delete_multiple_bodies(self, items):
        """Request deletion of multiple bodies"""
        body_ids = []
        for item in items:
            body_id = item.data(0, Qt.UserRole)
            if body_id is not None:
                body_ids.append(body_id)
        
        if body_ids:
            self.delete_multiple_bodies_requested.emit(body_ids)
    
    def _request_isolate_body(self, item):
        """Request isolation of a specific body"""
        body_id = item.data(0, Qt.UserRole)
        if body_id is not None:
            self.isolate_body_requested.emit(body_id)
    
    def _request_exit_isolation(self):
        """Request to exit isolation mode"""
        self.exit_isolation_requested.emit()
