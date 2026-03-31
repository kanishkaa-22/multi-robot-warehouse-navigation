# 🤖 AI Multi-Robot Warehouse Navigation System

## 📌 Overview

This project simulates multiple robots navigating inside a warehouse environment using **AI-based pathfinding algorithms**. It ensures **collision-free movement** using an advanced version of the **A* (A-Star) algorithm with Space-Time constraints**.

The system also includes a **BFS comparison** to demonstrate efficiency differences between algorithms.

---

## 🚀 Features

* ✅ Multi-robot coordination
* ✅ Space-Time A* path planning
* ✅ Collision avoidance (no overlapping or head-on collisions)
* ✅ BFS comparison for performance analysis
* ✅ Interactive GUI (Python Tkinter)
* ✅ Web-based simulation (HTML, CSS, JavaScript)
* ✅ Real-time animation
* ✅ Configurable grid and robot count

---

## 🧠 Algorithms Used

### 🔹 A* (A-Star)

* Uses **Manhattan Distance heuristic**
* Optimized for shortest path
* Supports **time-based reservations** to prevent collisions

### 🔹 BFS (Breadth-First Search)

* Guarantees shortest path
* Does **not handle collisions**
* Used for comparison with A*

---

## ⚙️ How It Works

1. Robots are assigned **start and goal positions**
2. Each robot plans its path using **Space-Time A***
3. A **reservation table** is used:

   ```
   (cell, time) → occupied
   ```
4. Robots move step-by-step without collisions

---

## 🖥️ Tech Stack

* **Python (Tkinter)** – Desktop GUI simulation
* **HTML, CSS, JavaScript** – Web-based simulation
* Algorithms: **A***, **BFS**

---

## 📂 Project Structure

```
AI-PROJECT/
│── warehouse_navigation.py     # Python Tkinter simulation
│── warehouse_navigation.html   # Web simulation
│── README.md
```

---

## ▶️ How to Run

### 🐍 Python Version

```bash
python warehouse_navigation.py
```

### 🌐 Web Version

* Open `warehouse_navigation.html` in your browser
  OR
* Use **Live Server** in VS Code

---

## 🎯 Use Cases

* Warehouse automation systems
* Robotics path planning
* Multi-agent AI systems
* Algorithm visualization

---

## ⚡ Key Concept

**Space-Time A*** ensures that each robot avoids conflicts by considering both:

* Position
* Time

This guarantees **collision-free navigation in multi-robot environments**.

---

## 📸 Demo

<img width="1919" height="867" alt="image" src="https://github.com/user-attachments/assets/73e67d4b-18b7-4c2a-b3cc-e1b2b576cb14" />



---

## ⭐ Future Improvements

* Dynamic obstacle handling
* Real-time replanning
* Machine learning-based optimization
* Integration with physical robots

---

## 📜 License

This project is developed for educational and academic purposes.
