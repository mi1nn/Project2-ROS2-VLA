CREATE TABLE IF NOT EXISTS item (
    item_id BIGSERIAL PRIMARY KEY,
    item_code VARCHAR(100) NOT NULL UNIQUE,
    item_name VARCHAR(200) NOT NULL
);

CREATE TABLE IF NOT EXISTS inventory (
    item_id BIGINT PRIMARY KEY
        REFERENCES item(item_id)
        ON DELETE CASCADE,
    quantity INTEGER NOT NULL DEFAULT 0
        CHECK (quantity >= 0),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);