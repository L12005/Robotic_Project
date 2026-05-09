from __future__ import annotations

import re


def parse_visual_ids_from_scene_text(
    scene_text: str,
    *,
    model_name: str,
    link_name: str,
) -> dict[str, int]:
    visual_ids: dict[str, int] = {}
    stack: list[dict[str, int | str | None]] = []
    name_pattern = re.compile(r'^name:\s*"(.*)"$')
    id_pattern = re.compile(r'^id:\s*(\d+)$')

    for raw_line in scene_text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.endswith('{'):
            stack.append({'type': line[:-1].strip(), 'name': None, 'id': None})
            continue
        if line == '}':
            if not stack:
                continue
            block = stack.pop()
            if block.get('type') == 'visual':
                current_model_name = _nearest_stack_name(stack, 'model')
                current_link_name = _nearest_stack_name(stack, 'link')
                visual_name = block.get('name')
                visual_id = block.get('id')
                if (
                    current_model_name == model_name
                    and current_link_name == link_name
                    and isinstance(visual_name, str)
                    and visual_name.startswith('led_segment_')
                    and isinstance(visual_id, int)
                    and visual_id > 0
                ):
                    visual_ids[visual_name] = visual_id
            continue

        if not stack:
            continue
        name_match = name_pattern.match(line)
        if name_match:
            stack[-1]['name'] = name_match.group(1)
            continue
        id_match = id_pattern.match(line)
        if id_match:
            stack[-1]['id'] = int(id_match.group(1))

    return visual_ids


def _nearest_stack_name(stack: list[dict[str, int | str | None]], block_type: str) -> str | None:
    for block in reversed(stack):
        if block.get('type') == block_type:
            name = block.get('name')
            return name if isinstance(name, str) else None
    return None
