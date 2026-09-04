BEGIN;

INSERT INTO item (item_id, item_code, item_name)
VALUES
    (0, '마스크', '마스크'),
    (1, '분유', '분유'),
    (2, '샴푸리필', '샴푸리필'),
    (3, '수세미', '수세미'),
    (4, '양갱', '양갱'),
    (5, '여행용티슈', '여행용티슈'),
    (6, '일회용숟가락', '일회용숟가락'),
    (7, '컵라면', '컵라면'),
    (8, '햄', '햄')
ON CONFLICT (item_id) DO UPDATE
SET item_code = EXCLUDED.item_code,
    item_name = EXCLUDED.item_name;

INSERT INTO inventory (item_id, quantity)
VALUES
    (0, 2),
    (1, 20),
    (2, 1),
    (3, 1),
    (4, 1),
    (5, 1),
    (6, 4),
    (7, 1),
    (8, 1)
ON CONFLICT (item_id) DO NOTHING;

SELECT setval(
    pg_get_serial_sequence('item', 'item_id'),
    (SELECT MAX(item_id) FROM item),
    true
);

COMMIT;
