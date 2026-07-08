
# AMR FINAL PROJECT


## Important Information

| Item | Details |
|------|---------|
| Assignment Release | 1 July 2026 |
| Due Date | **28 September 2026, 23:59 CET** |
| Repository Visibility | Public |
| Team Size | 3–4 students |
| Submission | Prepare a report with the format explained in class and Submit the GitHub repository URL on LEA |


# Getting Started

## Step 1

Click **Use this template** (green button at the top of this page).

## Step 2

Create a new repository using the following naming convention:

```
amr-team-<team_name>
```

Replace '<team_name>' with your desired team name.

## Step 3

Set the repository visibility to **Public** and create the repository.

## Step 4

Invite your team members as collaborators to the repository.

```
Settings
    ↓
Collaborators
    ↓
Add people
```

## Step 5

Clone your repository

example:

```bash
git clone https://github.com/amr-team-<team_name>.git
```

## Finally

Work collaboratively by splitting the tasks among team members and individually push your code to the repository.

## Important Note

- Team members work is evaluated based on your commit history, if  we do not see any commits from a team member then we cannot consider their contribution. 

- You can use issue boards and other tools to create issues and pull requests to manage your work and better showcase collaboration.

- Make sure you record almost every session because you need a working video to add into the report. Make sure to take screenshots, screenrecords etc to document your work in an effective manner.

- The robots in the lab are prone to issues so finish everything on simulation as fast as you can and start testing as soon as you can, do not wait until the last moment.

- Make sure to use only one branch to track all of your codes and also do not upload entire folders on to Github, use a gitignore and keep only required files on there.

- Write a nice Readme file on how to use the codes and also explain your approach for the tasks and also any challenges you faced, Feel free to modify this file.

- Ensure when leaving the lab you charge the robots for next team that is coming or if you are the last team unplug the robot, switch it off and then leave.

- Feel free to post any issues you faced on LEA, always refer to the documentation when in confusion and retrace your steps.
---

# AMR Project

## Project Objectives

The objective of this project is that you deploy some of the functionalities that were discussed during the course on a real robot platform. In particular, we want to have functionalities for path and motion planning, localisation, and environment exploration on the robot.

We will particularly use the Robile platform during the project; you are already familiar with this robot from the simulation you have been using throughout the semester as well as from the few practical lab sessions that we have had.

## Task Description

The project consists of three parts that are building on each other: (i) path and motion planning, (ii) localisation, and (iii) environment exploration.

### 1. Path and Motion Planning

You have already implemented a *potential field planner* in one of your assignments. In this first part of the project, you need to port your implementation to the real robot and ensure that it is working as well as it was in the simulated environment so that you can navigate towards global goals while avoiding obstacles. Then, integrate your potential field planner with a global path planner, namely first use a path planner (e.g. A*) to find a rough global trajectory of waypoints that the robot can follow to reach a goal and then use the potential field planner to navigate between the waypoints. This will make your potential field planner applicable to large environments, where it can navigate given an environment map.

### 2. Localisation

In one of the course lectures, we discussed Monte Carlo localisation as a practical solution to the robot localisation problem in an existing map. In this second part of the project, your objective is to implement your very own particle filter that you then integrate on the Robile. You should implement the simple version of the filter that we discussed in the lecture; however, if you have time and interest, you are free to additionally explore extensions / improvements to the algorithm, for example in the form of the adaptive Monte Carlo approach that we mentioned in the lecture.

### 3. Environment Exploration

The final objective of the project is to incorporate an environment exploration functionality to the robot. This will have to be combined with a SLAM component, namely you will need your exploration component to select poses to explore and a SLAM component that will take care of actually creating a map. The exploration algorithm should ideally select poses at the map fringe (i.e. poses that are at the boundary between the explored and unexplored region), but you are free to explore different pose selection strategies in your implementation.
