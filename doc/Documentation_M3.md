# Robohockey Documentation
**Authors:**</br>
Heinrich Fuhrmann 11940304 </br>
Thomas Schwabe 11918466</br>
Florian Pfleiderer 11771070</br>

**Version:** 
2023-05-09

## Milestone 3: Field Dimension Detection and Localisation

### Localisation Algorithm
The Algorithm used for localisation is based on the specific field setup, which is a known environment.
It s based on the fact, that if you know the distance between two poles and divide by the distance of two other poles, the proportions tell you
which poles they are and therefore you can calculate your own position on the field.

### Issues 
Spherical coordinates are in ISO 80000-2:2019 convention (used in physics, theta is angle from vertical z axis) and i used mathematical convention for algorithm...

### Contributions
- Heinrich Fuhrmann
    - Vector Calculations
    - state machine implementation
- Thomas Schwabe
    - documentation
    - implementation 
- Florian Pfleiderer
    - code refactoring
    - localisation algorithm
