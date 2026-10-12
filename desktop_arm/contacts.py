"""Contact classification shared by sampled preflight and physics execution."""


def fixed_scene_names(model):
    return {model.geom(i).name for i in range(model.ngeom)
            if model.geom_bodyid[i] == 0 and model.geom_contype[i] != 0 and model.geom(i).name}


def collision_pairs(model, data, *, carried_object=False):
    fixed = fixed_scene_names(model)
    base = model.body('link1').id
    pairs = set()
    for contact in data.contact:
        if contact.dist >= 0:
            continue
        ids = (int(contact.geom1), int(contact.geom2))
        names = tuple(model.geom(g).name for g in ids)
        if 'workpiece_geom' in names:
            if carried_object and any(n and n.startswith('obstacle_') for n in names):
                pairs.add(names)
        elif set(names) & fixed and any(model.geom_bodyid[g] not in (0, base) for g in ids):
            pairs.add(names)
    return pairs
