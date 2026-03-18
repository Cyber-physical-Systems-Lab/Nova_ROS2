from pathlib import Path


def test_tags_36h11_config_includes_tag_11_with_dedicated_size():
    text = (
        Path(__file__).resolve().parents[1] / 'config' / 'tags_36h11.yaml'
    ).read_text()

    assert 'tag_11' in text
    assert '0.095' in text
