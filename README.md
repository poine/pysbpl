# pysbpl
#### Forked from : https://github.com/poine/pysbpl
#### Edits Made By: Schmittle

Python3 bindings and utilities for SBPL ( Search Based Planning Library, see https://github.com/sbpl/sbpl ). Just the bindings for ARASTAR and EnvironmentNAVXYTHETALAT.


## Install & Run
1. Install [SBPL](https://github.com/sbpl/sbpl) from source.  
2. Clone this repo  
    ` git clone https://github.com/schmittlema/pysbpl.git` 
3. Build. Feel free to remove user to make install system wide
   - For Development: `pip install -e . --user`
   - For Use: `pip install . --user`
4. Test by running: `python3 examples/run_sbpl_xytheta.py`. 

If everything worked a visualization should appear

## To create new motion primitives
See `pysbpl/gen_mprim.py`. Run with:  
`python3 pysbpl/gen_mprim.py`  

A test.mprim file should be created


## Troubleshooting

**I properly installed SBPL but pip install is saying it doesnt exist:** This is likely a path linking issue. Add the path to `libsbpl.so` (usually `/usr/share/lib`) to `$LD_LIBRARY_PATH`

