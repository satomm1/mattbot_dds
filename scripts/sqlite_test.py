#!/usr/bin/env python3

import sqlite3
import os
import sys

def get_robot_goal_history(robot_id=2, limit=10, db_path='robot_data.db'):
    """
    Retrieve and display the most recent goals for a robot with the specified ID.
    
    Args:
        robot_id: The ID of the robot to query (default: 2)
        limit: Maximum number of goals to retrieve (default: 10)
        db_path: Path to the SQLite database file
    """
    # Check if database exists
    if not os.path.exists(db_path):
        print(f"Error: Database '{db_path}' not found.")
        return False
    
    try:
        # Connect to the database
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Get the most recent goals for the robot
        cursor.execute("""
            SELECT goal_id, x, y, theta, timestamp 
            FROM goals 
            WHERE robot_id = ? 
            ORDER BY timestamp DESC 
            LIMIT ?
        """, (robot_id, limit))
        
        goals = cursor.fetchall()
        
        if not goals:
            print(f"Robot {robot_id} has no goals set.")
            return False
        
        # Display the results
        print(f"Most recent {len(goals)} goal(s) for Robot {robot_id}:")
        print("-" * 60)
        print(f"{'Goal ID':<8} {'X':>8} {'Y':>8} {'Theta':>8} {'Timestamp':<20}")
        print("-" * 60)
        
        for goal in goals:
            goal_id, x, y, theta, timestamp = goal
            print(f"{goal_id:<8} {x:>8.2f} {y:>8.2f} {theta:>8.2f} {timestamp:<20}")
        
        # Display the latest goal separately
        latest_goal = goals[0]
        print("\nLatest Goal:")
        print(f"  Position: x={latest_goal[1]:.2f}, y={latest_goal[2]:.2f}, theta={latest_goal[3]:.2f} radians")
        print(f"  Set at: {latest_goal[4]}")
        
        return True
        
    except sqlite3.Error as e:
        print(f"Database error: {e}")
        return False
    
    finally:
        if conn:
            conn.close()

# If the table doesn't exist, create it
def ensure_goals_table_exists(db_path='robot_data.db'):
    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Check if the goals table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='goals'")
        if not cursor.fetchone():
            print("Creating goals table...")
            cursor.execute('''
            CREATE TABLE goals (
                goal_id INTEGER PRIMARY KEY,
                robot_id INTEGER NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                theta REAL NOT NULL,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
            ''')
            conn.commit()
            print("Goals table created successfully.")
        
        conn.close()
        return True
    except sqlite3.Error as e:
        print(f"Error ensuring goals table exists: {e}")
        return False

if __name__ == "__main__":
    # Use robot_data.db as the database name
    db_path = 'robot_data.db'
    
    # Get robot ID from command line if provided, otherwise use default (2)
    robot_id = 2
    if len(sys.argv) > 1:
        try:
            robot_id = int(sys.argv[1])
        except ValueError:
            print(f"Invalid robot ID: {sys.argv[1]}. Using default ID 2.")
    
    # Get limit from command line if provided, otherwise use default (10)
    limit = 10
    if len(sys.argv) > 2:
        try:
            limit = int(sys.argv[2])
        except ValueError:
            print(f"Invalid limit: {sys.argv[2]}. Using default limit 10.")
    
    # If database doesn't exist, create it with the goals table
    if not os.path.exists(db_path):
        print(f"Database '{db_path}' not found. Creating it...")
        try:
            conn = sqlite3.connect(db_path)
            conn.close()
            ensure_goals_table_exists(db_path)
            
            # Add sample goals for robot 2
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            # Insert a few sample goals
            sample_goals = [
                (2, 10.5, 20.3, 1.57),
                (2, 15.0, 25.0, 3.14),
                (2, 5.2, 8.7, 0.78),
                (2, 12.3, 18.9, 2.35)
            ]
            
            cursor.executemany("INSERT INTO goals (robot_id, x, y, theta) VALUES (?, ?, ?, ?)", sample_goals)
            conn.commit()
            conn.close()
            print(f"Added sample goals for robot {robot_id}")
        except sqlite3.Error as e:
            print(f"Failed to create database: {e}")
    else:
        # Make sure the goals table exists
        ensure_goals_table_exists(db_path)
    
    # Get the robot goal history
    get_robot_goal_history(robot_id, limit, db_path)