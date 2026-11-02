# Quarantined failure signatures run as xfail when mhseals_agents is on PYTHONPATH; a no-op otherwise.
pytest_plugins = ['mhseals_agents.pytest_quarantine'] if __import__('importlib.util').util.find_spec('mhseals_agents') else []
