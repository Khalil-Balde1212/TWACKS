# MuJoCo Simulation Starter

This folder is set up as a minimal MuJoCo Python project.

## 1) Use your workspace virtual environment

```bash
source .venv/bin/activate
```

If `.venv` does not exist yet, create it first:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

## 2) Install dependencies

```bash
pip install -r requirements.txt
```

## 3) Generating Model
This is mostly here for me. Unfortunately the lack of the API key won't allow you, random person, to load a new model
```bash
onshape-to-robot my-robot/
```

to test the model you (yes you!) can run
```bash
onshape-to-robot-mujoco my-robot/
```
